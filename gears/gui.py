"""tkinter front end: nine inputs, live derived values, and a 2D preview.

Everything here is presentation. The rules it enforces come from `validate`,
the numbers from `geometry`, and the drawing from `preview`; this module owns
only the widgets and the wiring between them.

Two things worth knowing about the wiring:

* **Every keystroke recomputes**, debounced by `REFRESH_DELAY_MS` so that
  typing "17" does not compute a set for "1" first. The whole geometry pass is
  well under a millisecond, so there is no reason to make the user press a
  button to see the consequences of an edit.
* **A SOLIDWORKS build runs on a worker thread.** COM is initialised on that
  thread by `SwSession`, and the worker formats its own report before handing
  it back, so the tkinter thread never touches a COM object - by the time the
  message arrives, `CoUninitialize` has already run.
"""

from __future__ import annotations

import dataclasses
import queue
import threading
import tkinter as tk
from dataclasses import dataclass
from pathlib import Path
from tkinter import filedialog, messagebox, ttk

from .bevel import preview as bevel_preview
from .bevel.geometry import compute_set as bevel_compute_set
from .bevel.params import BevelSetParams
from .bevel.validate import validate as bevel_validate
from .planetary import preview as planetary_preview
from .planetary.geometry import compute_set as planetary_compute_set
from .planetary.params import PlanetarySetParams
from .planetary.validate import validate as planetary_validate
from .spur import preview as spur_preview
from .spur.geometry import compute_set as spur_compute_set
from .spur.params import SpurSetParams
from .spur.validate import validate as spur_validate
from .preview import Scene, View, draw_scale_bar, draw_scene, write_dxf
from .validate import ValidationResult

REFRESH_DELAY_MS = 120
BUILD_POLL_MS = 150

ZOOM_MIN, ZOOM_MAX = 0.1, 40.0


@dataclass(frozen=True)
class Field:
    """One editable input, and how to read it back out of its widget.

    `choices` turns the row into a combobox instead of an entry. Only `hand`
    needs it, and it needs it because a helical pair meshes only if its members
    are wound opposite ways - which makes the pinion's hand a real input rather
    than a detail to leave at a default.
    """

    attr: str
    label: str
    kind: type
    unit: str = ""
    choices: tuple[str, ...] = ()

    def parse(self, text: str):
        text = text.strip()
        if not text:
            raise ValueError("is empty")
        if self.choices:
            if text not in self.choices:
                raise ValueError("must be one of " + ", ".join(self.choices))
            # A bool field is still a combobox - "external"/"internal" reads
            # better in a form than a checkbox labelled with a negation - so it
            # converts back here rather than leaving the caller to know.
            if self.kind is bool:
                return text == self.choices[1]
            return text
        try:
            value = float(text)
        except ValueError:
            raise ValueError(f"{text!r} is not a number") from None
        if self.kind is int:
            if abs(value - round(value)) > 1e-9:
                raise ValueError("must be a whole number")
            return int(round(value))
        return value

    def format(self, value) -> str:
        """The inverse of `parse`: what to write into the widget.

        Paired with `parse` deliberately, and tested as a round trip. A bool
        field goes through the choice labels rather than through `str(True)`,
        which is not one of them and would not parse back.
        """
        if self.choices:
            if self.kind is bool:
                return self.choices[1] if value else self.choices[0]
            return str(value)
        if self.kind is int:
            return str(value)
        return f"{value:g}"


BEVEL_FIELDS: tuple[Field, ...] = (
    Field("module", "Module (outer)", float, "mm"),
    Field("z1", "Pinion teeth z1", int),
    Field("z2", "Gear teeth z2", int),
    Field("pressure_angle", "Pressure angle", float, "deg"),
    Field("shaft_angle", "Shaft angle", float, "deg"),
    Field("spiral_angle", "Spiral angle (mean)", float, "deg"),
    Field("hand", "Hand (pinion)", str, "", ("right", "left")),
    Field("face_width", "Face width", float, "mm"),
    Field("bore", "Bore diameter", float, "mm"),
    Field("hub_thickness", "Hub thickness", float, "mm"),
    Field("min_root_thickness", "Min root thickness", float, "mm"),
)

SPUR_FIELDS: tuple[Field, ...] = (
    Field("module", "Normal module", float, "mm"),
    Field("z1", "Pinion teeth z1", int),
    Field("z2", "Gear teeth z2", int),
    Field("internal", "Arrangement", bool, "", ("external", "internal")),
    Field("pressure_angle", "Normal pressure angle", float, "deg"),
    Field("helix_angle", "Helix angle", float, "deg"),
    Field("hand", "Hand (pinion)", str, "", ("right", "left")),
    Field("face_width", "Face width", float, "mm"),
    Field("bore", "Bore diameter", float, "mm"),
    Field("hub_thickness", "Hub thickness", float, "mm"),
    Field("rim_thickness", "Ring rim thickness", float, "mm"),
)

PLANETARY_FIELDS: tuple[Field, ...] = (
    Field("module", "Normal module", float, "mm"),
    Field("z_sun", "Sun teeth", int),
    Field("z_planet", "Planet teeth", int),
    Field("n_planets", "Number of planets", int),
    Field("pressure_angle", "Normal pressure angle", float, "deg"),
    Field("helix_angle", "Helix angle", float, "deg"),
    Field("hand", "Hand (sun)", str, "", ("right", "left")),
    Field("face_width", "Face width", float, "mm"),
    Field("bore", "Sun bore diameter", float, "mm"),
    Field("hub_thickness", "Hub thickness", float, "mm"),
    Field("rim_thickness", "Ring rim thickness", float, "mm"),
)

# The order the rows are laid out in. Every field of every type appears once,
# and the rows belonging to the other type are hidden rather than destroyed - so
# switching type keeps whatever module and tooth counts were already typed.
#
# `hand` appears in both tuples and means the same thing in both, so it gets one
# row and one variable. That is the whole reason the rows are keyed by attribute
# name: a field two types share is shared for free, and a helix hand typed on a
# spur set is still there if you switch to a spiral bevel one.
def _ordered_fields() -> tuple[Field, ...]:
    seen: dict[str, Field] = {}
    for field in BEVEL_FIELDS + SPUR_FIELDS + PLANETARY_FIELDS:
        seen.setdefault(field.attr, field)
    order = (
        "module", "z1", "z2", "z_sun", "z_planet", "n_planets", "internal",
        "pressure_angle", "shaft_angle", "spiral_angle", "helix_angle", "hand",
        "face_width", "bore", "hub_thickness", "rim_thickness",
        "min_root_thickness",
    )
    return tuple(seen[name] for name in order)


ALL_FIELDS: tuple[Field, ...] = _ordered_fields()

# Fields `with_defaults` can size for us, and so the "Auto" button rewrites.
BEVEL_AUTO = ("face_width", "bore", "hub_thickness", "min_root_thickness")
SPUR_AUTO = ("face_width", "bore", "hub_thickness", "rim_thickness")
PLANETARY_AUTO = ("face_width", "bore", "hub_thickness", "rim_thickness")


def _bevel_status(p, geo) -> str:
    trace = (
        "straight teeth" if geo.trace is None
        else f"spiral {p.spiral_angle:g} deg {p.hand}"
    )
    return (
        f"m {p.module:g}   {p.z1}:{p.z2} teeth   ratio {p.ratio:.3f}:1   "
        f"shaft {p.shaft_angle:g} deg   {trace}   "
        f"cones {geo.pinion.pitch_angle_deg:.3f} / {geo.gear.pitch_angle_deg:.3f} deg"
    )


def _spur_status(p, geo) -> str:
    helix = (
        "straight teeth" if not p.helix_angle
        else f"helix {p.helix_angle:g} deg {p.hand}"
    )
    arrangement = "internal ring" if p.internal else "external"
    return (
        f"m_n {p.module:g}   {p.z1}:{p.z2} teeth   {arrangement}   "
        f"ratio {p.ratio:.3f}:1   {helix}   a {geo.centre_distance:.3f} mm   "
        f"contact {geo.total_contact_ratio:.3f}"
    )


def _planetary_status(p, geo) -> str:
    helix = (
        "straight teeth" if not p.helix_angle
        else f"helix {p.helix_angle:g} deg {p.hand}"
    )
    fits = "assembles" if p.assembly_remainder == 0 else "WILL NOT ASSEMBLE"
    return (
        f"m_n {p.module:g}   sun {p.z_sun} / {p.n_planets} planets of "
        f"{p.z_planet} / ring {p.z_ring}   {helix}   "
        f"a {geo.centre_distance:.3f} mm   "
        f"{p.ratio_carrier_to_sun:.3f}:1 with the ring held   {fits}"
    )


def _bevel_result_lines(result) -> list[str]:
    return [
        f"  shaft angle {result.measured_shaft_angle_deg:.4f} deg measured "
        f"({result.shaft_angle_error_deg:+.2e} deg error)",
    ]


def _spur_result_lines(result) -> list[str]:
    return [
        f"  centre distance {result.measured_centre_distance_mm:.4f} mm measured "
        f"({result.centre_distance_error_mm:+.2e} mm error)",
        f"  axes {result.measured_axis_angle_deg:.6f} deg apart (parallel is 0)",
    ]


def _planetary_result_lines(result) -> list[str]:
    return [
        f"  orbit radius {result.centre_distance_mm:.4f} mm, "
        f"{len(result.planets)} planets",
        f"  worst planet position error {result.worst_position_error_mm:+.2e} mm",
    ]


@dataclass(frozen=True)
class GearKind:
    """Everything the window needs to know in order to show one kind of gear.

    A registry rather than a branch in each method: the window is the same
    window either way, and every difference between the two is a value - which
    fields to show, which scenes exist, what the status line says, which builder
    to call.
    """

    key: str
    label: str
    params_cls: type
    compute_set: object
    validate: object
    preview: object
    fields: tuple[Field, ...]
    auto_fields: tuple[str, ...]
    auto_kwargs: tuple[str, ...]
    default_scene: str
    status: object
    result_lines: object
    builder: str            # the name to import out of `gears.sw`

    # The members this type has, in display order. A pair has two; a planetary
    # train has three, and the Member radio buttons follow rather than being
    # hard-coded to pinion and gear.
    members: tuple[str, ...] = ("pinion", "gear")

    # Which two fields carry the tooth counts. Both types of pair call them z1
    # and z2; a planetary set calls them z_sun and z_planet and derives the
    # third. Named here so switching type can carry the counts across without
    # knowing which type it is switching between.
    count_attrs: tuple[str, str] = ("z1", "z2")

    def counts(self, params) -> tuple[int, int]:
        """The two tooth counts of a params object of *this* kind."""
        return tuple(getattr(params, name) for name in self.count_attrs)

    def fallback(self):
        """A minimal set to fall back on before anything has been parsed."""
        return self.params_cls.with_defaults(1.0, 12, 12)


KINDS: dict[str, GearKind] = {
    "bevel": GearKind(
        key="bevel",
        label="Bevel",
        params_cls=BevelSetParams,
        compute_set=bevel_compute_set,
        validate=bevel_validate,
        preview=bevel_preview,
        fields=BEVEL_FIELDS,
        auto_fields=BEVEL_AUTO,
        # The spiral angle has to be carried into `with_defaults`, because the
        # face width it picks depends on it - a curved tooth takes the tighter
        # Gleason limit of 0.30*Ao. Without it, "Auto-size blank" on a spiral set
        # would hand back the straight set's face width and then warn about it.
        auto_kwargs=("pressure_angle", "shaft_angle", "spiral_angle", "hand"),
        default_scene="developed",
        status=_bevel_status,
        result_lines=_bevel_result_lines,
        builder="build_set",
    ),
    "spur": GearKind(
        key="spur",
        label="Spur",
        params_cls=SpurSetParams,
        compute_set=spur_compute_set,
        validate=spur_validate,
        preview=spur_preview,
        fields=SPUR_FIELDS,
        auto_fields=SPUR_AUTO,
        auto_kwargs=("pressure_angle", "helix_angle", "hand", "internal"),
        default_scene="transverse",
        status=_spur_status,
        result_lines=_spur_result_lines,
        builder="build_spur_set",
    ),
    "planetary": GearKind(
        key="planetary",
        label="Planetary",
        params_cls=PlanetarySetParams,
        compute_set=planetary_compute_set,
        validate=planetary_validate,
        preview=planetary_preview,
        fields=PLANETARY_FIELDS,
        auto_fields=PLANETARY_AUTO,
        auto_kwargs=("pressure_angle", "helix_angle", "hand", "n_planets"),
        default_scene="train",
        status=_planetary_status,
        result_lines=_planetary_result_lines,
        builder="build_planetary_set",
        members=("sun", "planet", "ring"),
        count_attrs=("z_sun", "z_planet"),
    ),
}

TEXT_COLOURS = {
    "error": "#b0202a",
    "warn": "#8a6100",
    "ok": "#1f7a3d",
    "info": "#555555",
}


class App(ttk.Frame):
    """The whole window."""

    def __init__(self, master: tk.Misc):
        super().__init__(master, padding=8)

        # One variable per field name across both types. Shared on purpose:
        # switching type keeps the module and tooth counts already typed.
        self.vars = {f.attr: tk.StringVar() for f in ALL_FIELDS}
        self.kind_key = tk.StringVar(value="bevel")
        self.member = tk.StringVar(value="pinion")
        self.scene_key = tk.StringVar(value=KINDS["bevel"].default_scene)

        self._loading = False           # suppress refresh while writing Entries
        self._pending_refresh: str | None = None
        self._params = None
        self._geo = None
        self._scene: Scene | None = None
        self._validation = ValidationResult()

        self._zoom = 1.0
        self._pan = [0.0, 0.0]
        self._drag: tuple[float, float] | None = None

        self._build_queue: queue.Queue = queue.Queue()
        self._build_thread: threading.Thread | None = None
        self._previous_kind = KINDS[self.kind_key.get()]

        self._make_widgets()
        for var in self.vars.values():
            var.trace_add("write", self._on_input_change)

        self._apply_kind()
        self.set_params(BevelSetParams.with_defaults(2.0, 17, 43))

    # -- the active gear type -----------------------------------------------

    @property
    def kind(self) -> GearKind:
        return KINDS[self.kind_key.get()]

    def on_kind_change(self) -> None:
        """Switch gear type: re-show the right rows, scenes and defaults.

        The parameters are rebuilt through `with_defaults` rather than carried
        across field by field. The shared names mean the same thing in both types
        - module, tooth counts, pressure angle - but the sizing rules do not, and
        a face width that suited a bevel set is not the one a helical spur set
        wants. Auto-sizing is the honest answer and it is one the user can
        immediately overtype.
        """
        previous = self._previous_kind
        kind = self.kind
        self._apply_kind()

        # The previous type's parameters, which still carry the inputs every
        # type shares. Reading the widgets instead would fail: the fields only
        # the new type has are still empty.
        #
        # The counts are read through the kind they were *written* by, because
        # the field names differ - a spur set has z1 and z2 where a planetary
        # set has z_sun and z_planet - and the object in hand is still the old
        # type's.
        base = self._params
        if base is None or not isinstance(base, previous.params_cls):
            base, previous = kind.fallback(), kind
        first, second = previous.counts(base)
        self.set_params(kind.params_cls.with_defaults(base.module, first, second))

    def _apply_kind(self) -> None:
        """Show the active type's rows and scenes; hide the other type's."""
        kind = self.kind
        shown = {f.attr: f for f in kind.fields}

        for attr, row in self.field_rows.items():
            field = shown.get(attr)
            if field is None:
                for widget in row:
                    widget.grid_remove()
            else:
                for widget in row:
                    widget.grid()
                row[0].configure(text=field.label)
                row[2].configure(text=field.unit)

        for button in self.member_buttons.values():
            button.pack_forget()
        for value in kind.members:
            self.member_buttons[value].pack(side="left", padx=(4, 0))
        if self.member.get() not in kind.members:
            self.member.set(kind.members[0])

        wanted = [key for key, _ in kind.preview.SCENE_LABELS]
        for button in self.scene_buttons.values():
            button.pack_forget()
        for key in wanted:
            self.scene_buttons[key].pack(side="left", padx=(4, 0))
        if self.scene_key.get() not in wanted:
            self.scene_key.set(kind.default_scene)

        # Remembered so the *next* switch can read the counts out of the params
        # object this one leaves behind, whose field names are this type's.
        self._previous_kind = kind

    # -- widgets ------------------------------------------------------------

    def _make_widgets(self) -> None:
        self.columnconfigure(1, weight=1)
        self.rowconfigure(0, weight=1)

        left = ttk.Frame(self)
        left.grid(row=0, column=0, sticky="ns")
        left.rowconfigure(2, weight=1)

        right = ttk.Frame(self)
        right.grid(row=0, column=1, sticky="nsew", padx=(10, 0))
        right.columnconfigure(0, weight=1)
        right.rowconfigure(2, weight=3)
        right.rowconfigure(3, weight=2)

        self._make_inputs(left)
        self._make_actions(left)
        self._make_messages(left)
        self._make_toolbar(right)
        self._make_canvas(right)
        self._make_readout(right)

        self.status = ttk.Label(self, text="", anchor="w", relief="sunken", padding=(6, 3))
        self.status.grid(row=1, column=0, columnspan=2, sticky="ew", pady=(8, 0))

    def _make_inputs(self, parent: ttk.Frame) -> None:
        box = ttk.LabelFrame(parent, text="Inputs", padding=8)
        box.grid(row=0, column=0, sticky="ew")

        # A row per field of *either* type, hidden by `_apply_kind` when it does
        # not belong to the one on show.
        self.entries: dict[str, tk.Misc] = {}
        self.field_rows: dict[str, tuple] = {}
        for row, field in enumerate(ALL_FIELDS):
            label = ttk.Label(box, text=field.label)
            label.grid(row=row, column=0, sticky="w", pady=2)
            if field.choices:
                widget = ttk.Combobox(
                    box,
                    textvariable=self.vars[field.attr],
                    values=list(field.choices),
                    width=9,
                    state="readonly",
                )
            else:
                widget = tk.Entry(
                    box,
                    textvariable=self.vars[field.attr],
                    width=11,
                    justify="right",
                    relief="solid",
                    borderwidth=1,
                    highlightthickness=0,
                )
            widget.grid(row=row, column=1, sticky="e", padx=(10, 4), pady=2)
            unit = ttk.Label(box, text=field.unit, width=4)
            unit.grid(row=row, column=2, sticky="w")
            self.entries[field.attr] = widget
            self.field_rows[field.attr] = (label, widget, unit)

        ttk.Button(box, text="Auto-size blank", command=self.auto_size).grid(
            row=len(ALL_FIELDS), column=0, columnspan=3, sticky="ew", pady=(8, 0)
        )

    def _make_actions(self, parent: ttk.Frame) -> None:
        box = ttk.LabelFrame(parent, text="Presets", padding=8)
        box.grid(row=1, column=0, sticky="ew", pady=(8, 0))
        box.columnconfigure(0, weight=1)
        box.columnconfigure(1, weight=1)
        ttk.Button(box, text="Load...", command=self.load_preset_dialog).grid(
            row=0, column=0, sticky="ew", padx=(0, 3)
        )
        ttk.Button(box, text="Save...", command=self.save_preset_dialog).grid(
            row=0, column=1, sticky="ew", padx=(3, 0)
        )

    def _make_messages(self, parent: ttk.Frame) -> None:
        box = ttk.LabelFrame(parent, text="Validation", padding=6)
        box.grid(row=2, column=0, sticky="nsew", pady=(8, 0))
        box.columnconfigure(0, weight=1)
        box.rowconfigure(0, weight=1)

        self.messages = tk.Text(
            box, width=38, height=14, wrap="word", relief="flat",
            background="#fbfbfb", padx=4, pady=4,
        )
        self.messages.grid(row=0, column=0, sticky="nsew")
        scroll = ttk.Scrollbar(box, orient="vertical", command=self.messages.yview)
        scroll.grid(row=0, column=1, sticky="ns")
        self.messages.configure(yscrollcommand=scroll.set, state="disabled")

        for tag, colour in TEXT_COLOURS.items():
            self.messages.tag_configure(tag, foreground=colour)
        self.messages.tag_configure("head", font=("TkDefaultFont", 9, "bold"))

    def _make_toolbar(self, parent: ttk.Frame) -> None:
        bar = ttk.Frame(parent)
        bar.grid(row=0, column=0, sticky="ew")

        ttk.Label(bar, text="Type").pack(side="left")
        for key, kind in KINDS.items():
            ttk.Radiobutton(
                bar, text=kind.label, value=key, variable=self.kind_key,
                command=self.on_kind_change,
            ).pack(side="left", padx=(4, 0))

        ttk.Separator(bar, orient="vertical").pack(side="left", fill="y", padx=10)

        ttk.Label(bar, text="Member").pack(side="left")
        # One button per member name of any type; `_apply_kind` packs the set
        # the active type actually has. A pair shows Pinion and Gear, a
        # planetary train shows Sun, Planet and Ring.
        self.member_buttons: dict[str, ttk.Radiobutton] = {}
        for kind in KINDS.values():
            for value in kind.members:
                if value in self.member_buttons:
                    continue
                self.member_buttons[value] = ttk.Radiobutton(
                    bar, text=value.capitalize(), value=value,
                    variable=self.member, command=self._on_view_change,
                )

        ttk.Separator(bar, orient="vertical").pack(side="left", fill="y", padx=10)

        ttk.Label(bar, text="View").pack(side="left")
        # One button per scene of either type; `_apply_kind` packs the right set.
        self.scene_buttons: dict[str, ttk.Radiobutton] = {}
        for kind in KINDS.values():
            for value, label in kind.preview.SCENE_LABELS:
                if value in self.scene_buttons:
                    continue
                self.scene_buttons[value] = ttk.Radiobutton(
                    bar, text=label, value=value, variable=self.scene_key,
                    command=self._on_view_change,
                )

        self.build_button = ttk.Button(
            bar, text="Build in SOLIDWORKS", command=self.build_in_solidworks
        )
        self.build_button.pack(side="right")
        ttk.Button(bar, text="Export CSV", command=self.export_csv).pack(
            side="right", padx=(0, 6)
        )
        ttk.Button(bar, text="Export DXF", command=self.export_dxf).pack(
            side="right", padx=(0, 6)
        )
        ttk.Button(bar, text="Fit", command=self.fit_view).pack(side="right", padx=(0, 6))

        self.scene_title = ttk.Label(parent, text="", anchor="w", foreground="#444444")
        self.scene_title.grid(row=1, column=0, sticky="ew", pady=(6, 2))

    def _make_canvas(self, parent: ttk.Frame) -> None:
        self.canvas = tk.Canvas(
            parent, width=640, height=430, background="white",
            highlightthickness=1, highlightbackground="#c9c9c9",
        )
        self.canvas.grid(row=2, column=0, sticky="nsew")
        self.canvas.bind("<Configure>", lambda _e: self._redraw())
        self.canvas.bind("<ButtonPress-1>", self._on_drag_start)
        self.canvas.bind("<B1-Motion>", self._on_drag)
        self.canvas.bind("<ButtonRelease-1>", lambda _e: setattr(self, "_drag", None))
        self.canvas.bind("<Double-Button-1>", lambda _e: self.fit_view())
        self.canvas.bind("<MouseWheel>", self._on_wheel)

    def _make_readout(self, parent: ttk.Frame) -> None:
        box = ttk.LabelFrame(parent, text="Derived values (read-only)", padding=6)
        box.grid(row=3, column=0, sticky="nsew", pady=(8, 0))
        box.columnconfigure(0, weight=1)
        box.rowconfigure(0, weight=1)

        # Three value columns. A gear pair fills two and leaves the third
        # empty; `_refresh_readout` hides it entirely for such a type, so the
        # window looks exactly as it did before planetary sets existed.
        columns = ("quantity", "first", "second", "third", "unit")
        self.readout = ttk.Treeview(box, columns=columns, show="headings", height=11)
        for column, heading, width, anchor in (
            ("quantity", "Quantity", 210, "w"),
            ("first", "Pinion", 105, "e"),
            ("second", "Gear", 105, "e"),
            ("third", "", 105, "e"),
            ("unit", "Unit", 45, "w"),
        ):
            self.readout.heading(column, text=heading)
            self.readout.column(column, width=width, anchor=anchor, stretch=(column == "quantity"))
        self.readout.grid(row=0, column=0, sticky="nsew")

        scroll = ttk.Scrollbar(box, orient="vertical", command=self.readout.yview)
        scroll.grid(row=0, column=1, sticky="ns")
        self.readout.configure(yscrollcommand=scroll.set)
        self.readout.tag_configure(
            "header", font=("TkDefaultFont", 9, "bold"), background="#eaeff4"
        )

    # -- parameters ---------------------------------------------------------

    def set_params(self, p) -> None:
        """Write a parameter set into the inputs and refresh once."""
        self._loading = True
        try:
            for field in self.kind.fields:
                self.vars[field.attr].set(field.format(getattr(p, field.attr)))
        finally:
            self._loading = False
        self.refresh()

    def read_params(self):
        """Parse the inputs. Returns (params or None, per-field parse errors)."""
        kind = self.kind
        values: dict[str, object] = {}
        problems: list[str] = []
        for field in kind.fields:
            try:
                values[field.attr] = field.parse(self.vars[field.attr].get())
            except ValueError as exc:
                problems.append(f"{field.label}: {exc}")
        if problems:
            return None, problems

        base = self._params
        if base is None or not isinstance(base, kind.params_cls):
            base = kind.fallback()
        return dataclasses.replace(base, **values), []

    def auto_size(self) -> None:
        """Reset the blank dimensions to what `with_defaults` would pick."""
        kind = self.kind
        p, problems = self.read_params()
        if p is None:
            self._show_messages(problems, None)
            return
        sized = kind.params_cls.with_defaults(
            p.module, p.z1, p.z2,
            **{name: getattr(p, name) for name in kind.auto_kwargs},
        )
        self.set_params(
            dataclasses.replace(
                p, **{a: getattr(sized, a) for a in kind.auto_fields}
            )
        )

    # -- refresh cycle ------------------------------------------------------

    def _on_input_change(self, *_args) -> None:
        if self._loading:
            return
        if self._pending_refresh is not None:
            self.after_cancel(self._pending_refresh)
        self._pending_refresh = self.after(REFRESH_DELAY_MS, self._refresh_now)

    def _refresh_now(self) -> None:
        self._pending_refresh = None
        self.refresh()

    def refresh(self) -> None:
        """Parse, validate, recompute, redraw. Safe to call at any time."""
        p, problems = self.read_params()
        for field in self.kind.fields:
            if field.choices:
                continue            # a readonly combobox cannot be mistyped
            bad = any(msg.startswith(field.label + ":") for msg in problems)
            self.entries[field.attr].configure(
                foreground=TEXT_COLOURS["error"] if bad else "black"
            )

        if p is None:
            self._params, self._geo, self._scene = None, None, None
            self._validation = ValidationResult()
            self._clear_readout()
            self._show_messages(problems, None)
            self._redraw()
            self._set_status(f"{len(problems)} input(s) cannot be read")
            self._update_buttons()
            return

        self._params = p
        self._validation = self.kind.validate(p)

        geo_error: str | None = None
        try:
            self._geo = self.kind.compute_set(p)
        except Exception as exc:                     # nonsense inputs, not a bug
            self._geo = None
            geo_error = f"geometry could not be computed: {exc}"

        self._show_messages(problems, self._validation, geo_error)
        if self._geo is None:
            self._clear_readout()
            self._scene = None
        else:
            self._refresh_readout(self._geo)
            self._refresh_scene(reset_view=False)
        self._redraw()
        self._update_buttons()
        self._set_status(self._status_text())

    def _status_text(self) -> str:
        errors = len(self._validation.errors)
        warnings = len(self._validation.warnings)
        if self._geo is None:
            return "no geometry"
        head = self.kind.status(self._params, self._geo)
        verdict = (
            "buildable" if not errors else f"{errors} error(s) - build blocked"
        )
        if warnings:
            verdict += f", {warnings} warning(s)"
        return f"{head}   |   {verdict}"

    def _update_buttons(self) -> None:
        busy = self._build_thread is not None and self._build_thread.is_alive()
        ready = self._geo is not None and self._validation.ok and not busy
        self.build_button.state(["!disabled"] if ready else ["disabled"])

    # -- messages -----------------------------------------------------------

    def _show_messages(
        self,
        parse_errors: list[str],
        result: ValidationResult | None,
        geo_error: str | None = None,
    ) -> None:
        self.messages.configure(state="normal")
        self.messages.delete("1.0", "end")

        def write(text: str, *tags: str) -> None:
            self.messages.insert("end", text + "\n", tags)

        if parse_errors:
            write("Inputs", "head")
            for msg in parse_errors:
                write(f"  {msg}", "error")

        if geo_error:
            write("Geometry", "head")
            write(f"  {geo_error}", "error")

        if result is not None:
            if result.errors:
                write("Errors", "head")
                for issue in result.errors:
                    write(f"  {issue}", "error")
            if result.warnings:
                write("Warnings", "head")
                for issue in result.warnings:
                    write(f"  {issue}", "warn")
            if result.ok and not result.warnings and not parse_errors and not geo_error:
                write("No errors, no warnings.", "ok")
            elif result.ok and not parse_errors and not geo_error:
                write("")
                write("Buildable - the warnings above are advisory.", "ok")

        self.messages.configure(state="disabled")

    def append_message(self, lines, tag: str = "info") -> None:
        """Add a block of text under whatever is already in the messages pane."""
        self.messages.configure(state="normal")
        self.messages.insert("end", "\n")
        for line in lines:
            self.messages.insert("end", line + "\n", tag)
        self.messages.see("end")
        self.messages.configure(state="disabled")

    def _set_status(self, text: str) -> None:
        self.status.configure(text=text)

    # -- readout ------------------------------------------------------------

    def _clear_readout(self) -> None:
        self.readout.delete(*self.readout.get_children())

    def _refresh_readout(self, geo) -> None:
        scroll_top = self.readout.yview()[0]
        self._clear_readout()

        rows = self.kind.preview.derived_rows(geo)
        # A pair never sets the third column, so it is displayed only for a type
        # that has something to put there rather than as a permanent blank.
        wide = any(row.third for row in rows)
        self.readout.configure(
            displaycolumns=("quantity", "first", "second", "third", "unit")
            if wide else ("quantity", "first", "second", "unit")
        )
        heads = (
            ("Sun", "Planet", "Ring") if wide else ("Pinion", "Gear", "")
        )
        for column, heading in zip(("first", "second", "third"), heads):
            self.readout.heading(column, text=heading)

        for row in rows:
            self.readout.insert(
                "", "end",
                values=(row.label, *row.values, row.unit),
                tags=("header",) if row.header else (),
            )
        self.readout.yview_moveto(scroll_top)

    # -- preview ------------------------------------------------------------

    def _on_view_change(self) -> None:
        self._refresh_scene(reset_view=True)
        self._redraw()

    def _refresh_scene(self, reset_view: bool) -> None:
        if self._geo is None:
            self._scene = None
            return
        try:
            self._scene = self.kind.preview.build_scene(
                self._geo, self.member.get(), self.scene_key.get()
            )
        except Exception as exc:                     # unbuildable profile, not a bug
            self._scene = None
            self.scene_title.configure(text=f"preview unavailable: {exc}")
            return
        self.scene_title.configure(text=self._scene.title)
        if reset_view:
            self._zoom, self._pan = 1.0, [0.0, 0.0]

    def fit_view(self) -> None:
        self._zoom, self._pan = 1.0, [0.0, 0.0]
        self._redraw()

    def _view(self, width: float, height: float) -> View:
        return View.fit(
            self._scene.bounds(), width, height,
            zoom=self._zoom, pan=tuple(self._pan),
        )

    def _redraw(self) -> None:
        self.canvas.delete("all")
        width = self.canvas.winfo_width()
        height = self.canvas.winfo_height()
        if width <= 1 or height <= 1:
            return

        if self._scene is None:
            self.canvas.create_text(
                width / 2, height / 2,
                text="no preview - fix the inputs listed under Validation",
                fill="#888888",
            )
            return

        view = self._view(width, height)
        draw_scene(self.canvas, self._scene, view)
        draw_scale_bar(self.canvas, view, width, height)

    def _on_drag_start(self, event) -> None:
        self._drag = (event.x, event.y)

    def _on_drag(self, event) -> None:
        if self._drag is None:
            return
        x0, y0 = self._drag
        self._pan[0] += event.x - x0
        self._pan[1] += event.y - y0
        self._drag = (event.x, event.y)
        self._redraw()

    def _on_wheel(self, event) -> None:
        """Zoom about the cursor, so the point under it stays put."""
        if self._scene is None:
            return
        width, height = self.canvas.winfo_width(), self.canvas.winfo_height()
        before = self._view(width, height)
        anchor = before.from_canvas(event.x, event.y)

        step = 1.15 ** (event.delta / 120.0 if event.delta else 1.0)
        self._zoom = max(ZOOM_MIN, min(ZOOM_MAX, self._zoom * step))

        after = self._view(width, height)
        moved = after.to_canvas(*anchor)
        self._pan[0] += event.x - moved[0]
        self._pan[1] += event.y - moved[1]
        self._redraw()

    # -- files --------------------------------------------------------------

    def _default_stem(self) -> str:
        p = self._params
        if p is None:
            return "bevel"
        module = f"{p.module:g}".replace(".", "p")
        return f"{self.member.get()}_m{module}_z{p.z1}x{p.z2}"

    def export_dxf(self) -> None:
        if self._scene is None:
            messagebox.showinfo("Export DXF", "There is nothing to export yet.")
            return
        path = filedialog.asksaveasfilename(
            title="Export the current view as DXF",
            defaultextension=".dxf",
            initialfile=f"{self._default_stem()}_{self.scene_key.get()}.dxf",
            filetypes=[("DXF", "*.dxf"), ("All files", "*.*")],
        )
        if not path:
            return
        written = write_dxf(path, self._scene)
        self._set_status(f"wrote {written}")

    def export_csv(self) -> None:
        if self._geo is None:
            messagebox.showinfo("Export CSV", "There is nothing to export yet.")
            return
        path = filedialog.asksaveasfilename(
            title="Export both tooth-space sections as CSV",
            defaultextension=".csv",
            initialfile=f"{self._default_stem()}_sections.csv",
            filetypes=[("CSV", "*.csv"), ("All files", "*.*")],
        )
        if not path:
            return
        written = self.kind.preview.write_csv(path, self._geo, self.member.get())
        self._set_status(f"wrote {written}")

    def load_preset_dialog(self) -> None:
        path = filedialog.askopenfilename(
            title="Load parameters",
            defaultextension=".json",
            filetypes=[("JSON", "*.json"), ("All files", "*.*")],
        )
        if path:
            self.load_preset(Path(path))

    def load_preset(self, path: Path) -> None:
        try:
            self.set_params(self.kind.params_cls.from_json(path))
        except Exception as exc:
            messagebox.showerror("Load parameters", f"Could not load {path}:\n\n{exc}")
            return
        self._set_status(f"loaded {path}")

    def save_preset_dialog(self) -> None:
        if self._params is None:
            messagebox.showinfo("Save parameters", "The inputs are not valid yet.")
            return
        path = filedialog.asksaveasfilename(
            title="Save parameters",
            defaultextension=".json",
            initialfile=f"{self._default_stem()}.json",
            filetypes=[("JSON", "*.json"), ("All files", "*.*")],
        )
        if not path:
            return
        self._params.to_json(path)
        self._set_status(f"wrote {path}")

    # -- SOLIDWORKS ---------------------------------------------------------

    def build_in_solidworks(self) -> None:
        """Kick off a build on a worker thread; results arrive by queue."""
        if self._geo is None or not self._validation.ok:
            messagebox.showinfo(
                "Build", "Fix the errors under Validation before building."
            )
            return
        if self._build_thread is not None and self._build_thread.is_alive():
            return

        out_dir = filedialog.askdirectory(
            title="Where should the parts and assembly be saved?",
            initialdir=str(Path.cwd()),
            mustexist=False,
        )
        if not out_dir:
            return

        geo = self._geo
        kind = self.kind
        self._build_thread = threading.Thread(
            target=self._build_worker,
            args=(geo, Path(out_dir), kind.builder, kind.result_lines),
            daemon=True,
        )
        self._build_thread.start()
        self._update_buttons()
        self._set_status("building in SOLIDWORKS...")
        self.append_message(["Build started - SOLIDWORKS is working."], "info")
        self.after(BUILD_POLL_MS, self._poll_build)

    def _build_worker(self, geo, out_dir: Path, builder: str, result_lines) -> None:
        """Runs off the UI thread. Formats its report before handing it back.

        Nothing COM-flavoured may cross back to the tkinter thread: by the time
        the queued message is read, `SwSession.__exit__` has already called
        `CoUninitialize`, so any surviving interface pointer would be dead.
        """
        try:
            from . import sw

            SwSession = sw.SwSession
            build = getattr(sw, builder)
        except ImportError as exc:
            self._build_queue.put(
                ("error", [f"pywin32 is not available: {exc}",
                           "Install it with: .venv\\Scripts\\pip install pywin32"])
            )
            return

        try:
            with SwSession() as session:
                result = build(session, geo, out_dir)
                lines = self._format_result(result, result_lines)
        except Exception as exc:
            self._build_queue.put(("error", [f"{type(exc).__name__}: {exc}"]))
            return
        self._build_queue.put(("done", lines))

    @staticmethod
    def _format_result(result, result_lines) -> list[str]:
        lines = ["Build finished."]
        # `parts` is what every result object offers: a pair returns its two
        # members, a planetary train its sun, one planet and its ring. Reading
        # the attribute rather than a fixed pair is what lets one formatter
        # serve a set with three members in it.
        for part in result.parts:
            lines.append(
                f"  {part.member}: {part.teeth} teeth, {part.body_count} body, "
                f"{part.face_count} faces"
            )
            if part.path:
                lines.append(f"    {part.path}")
        lines.extend(result_lines(result))
        lines.append(f"  gear clocked {result.clocking_deg:.4f} deg")
        if result.mates:
            lines.append(
                f"  {len(result.mates)} mates, "
                + ", ".join(
                    f"gear mate {num:g}:{den:g}" for num, den in result.gear_ratios
                )
            )
            lines.append(
                "  the set turns - drag either member in SOLIDWORKS"
                if result.articulates
                else "  WARNING: the set came back constrained, so it will not turn"
            )
        if result.interference_count < 0:
            lines.append("  interference: detection unavailable")
        elif result.interference_count == 0:
            lines.append("  interference: none")
        else:
            lines.append(
                f"  interference: {result.interference_count} region(s), "
                f"{result.interference_volume_mm3:.4f} mm3"
            )
        if result.assembly_path:
            lines.append(f"    {result.assembly_path}")
        return lines

    def _poll_build(self) -> None:
        try:
            kind, lines = self._build_queue.get_nowait()
        except queue.Empty:
            if self._build_thread is not None and self._build_thread.is_alive():
                self.after(BUILD_POLL_MS, self._poll_build)
            return

        self.append_message(lines, "ok" if kind == "done" else "error")
        self._set_status(lines[0])
        self._update_buttons()
        if kind == "error":
            messagebox.showerror("Build failed", "\n".join(lines))


def main(argv=None) -> int:
    root = tk.Tk()
    root.title("Bevel gear generator")
    root.minsize(1060, 720)

    app = App(root)
    app.pack(fill="both", expand=True)

    if argv:
        app.load_preset(Path(argv[0]))

    root.mainloop()
    return 0


if __name__ == "__main__":
    import sys

    sys.exit(main(sys.argv[1:]))
