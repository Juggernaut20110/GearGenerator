"""The terminal entry point: dispatch, flag hygiene, exit codes."""

from __future__ import annotations

import pytest

from gears.__main__ import main

ANCHOR = ["--module", "2", "--z1", "17", "--z2", "43"]


def run(argv, capsys):
    code = main(argv)
    captured = capsys.readouterr()
    return code, captured.out, captured.err


# --- dispatch --------------------------------------------------------------


def test_bevel_is_the_default_type(capsys):
    """The invocation that worked before there was a choice must still work."""
    code, out, _ = run(ANCHOR, capsys)
    assert code == 0
    assert "outer cone distance Ao" in out


def test_spur_is_selected_by_type(capsys):
    code, out, _ = run(["--type", "spur"] + ANCHOR, capsys)
    assert code == 0
    assert "centre distance a" in out
    assert "cone" not in out


@pytest.mark.parametrize("gear_type", ["bevel", "spur"])
def test_every_type_prints_the_same_three_sections(gear_type, capsys):
    _, out, _ = run(["--type", gear_type] + ANCHOR, capsys)
    for heading in ("INPUT", "SET", "MEMBERS", "TOOTH SPACE SECTION", "BLANK OUTLINE"):
        assert heading in out


# --- flag hygiene ----------------------------------------------------------


@pytest.mark.parametrize(
    "argv,offender",
    [
        (["--type", "spur", "--sigma", "90"], "--sigma"),
        (["--type", "spur", "--min-root", "0.5"], "--min-root"),
        (["--type", "spur", "--end", "inner"], "--end"),
        (["--type", "bevel", "--beta", "15"], "--beta"),
        (["--type", "bevel", "--backlash", "0.1"], "--backlash"),
        (["--type", "spur", "--spiral", "35"], "--spiral"),
        (["--type", "spur", "--cutter-radius", "40"], "--cutter-radius"),
    ],
)
def test_a_flag_from_the_other_type_is_refused(argv, offender, capsys):
    """Silently ignoring it would leave the user believing something untrue."""
    with pytest.raises(SystemExit) as exit_info:
        main(argv + ANCHOR)
    assert exit_info.value.code == 2
    assert offender in capsys.readouterr().err


@pytest.mark.parametrize("gear_type", ["bevel", "spur"])
def test_hand_belongs_to_both_types_and_is_refused_by_neither(gear_type):
    """It used to be a spur flag; spiral bevel gave it a second owner.

    Both types mean the same thing by it - which way the pinion's teeth wind,
    with the gear taking the other - so it moved into the shared parser. A
    regression here would show up as `--hand` being rejected on whichever type
    lost it, which is exactly the silent-wrong-answer the guard exists to stop.
    """
    assert main(["--type", gear_type, "--hand", "left"] + ANCHOR) == 0


@pytest.mark.parametrize("form", ["--beta 15", "--beta=15"])
def test_the_guard_catches_both_spellings_of_a_flag(form, capsys):
    with pytest.raises(SystemExit):
        main(form.split() + ANCHOR)


@pytest.mark.parametrize("gear_type", ["bevel", "spur"])
@pytest.mark.parametrize("flag", ["--alpha", "--face-width", "--bore", "--hub"])
def test_shared_flags_are_accepted_by_every_type(gear_type, flag, capsys):
    value = "20" if flag == "--alpha" else "12"
    code, _, _ = run(["--type", gear_type, flag, value] + ANCHOR, capsys)
    assert code == 0


# --- validation and exit codes ---------------------------------------------


@pytest.mark.parametrize("gear_type", ["bevel", "spur"])
def test_an_invalid_set_exits_nonzero_and_prints_nothing_to_stdout(gear_type, capsys):
    code, out, err = run(
        ["--type", gear_type, "--module", "2", "--z1", "17", "--z2", "43",
         "--bore", "400"],
        capsys,
    )
    assert code == 1
    assert out == ""
    assert "ERROR" in err


@pytest.mark.parametrize("gear_type", ["bevel", "spur"])
def test_warnings_go_to_stderr_so_the_report_pipes_clean(gear_type, capsys):
    code, out, err = run(["--type", gear_type] + ANCHOR, capsys)
    assert code == 0
    assert "WARNING" in err
    assert "WARNING" not in out


# --- the csv dump ----------------------------------------------------------


@pytest.mark.parametrize("gear_type", ["bevel", "spur"])
def test_csv_holds_the_developed_and_placed_points(gear_type, tmp_path, capsys):
    path = tmp_path / "nested" / "section.csv"
    code, out, _ = run(["--type", gear_type, "--csv", str(path)] + ANCHOR, capsys)
    assert code == 0

    lines = path.read_text(encoding="utf-8").strip().splitlines()
    assert lines[0] == "x_dev,y_dev,x,y,z"
    assert len(lines) > 20
    assert all(len(line.split(",")) == 5 for line in lines[1:])
    assert f"wrote {len(lines) - 1} points" in out


def test_a_straight_spur_section_lies_in_one_plane(tmp_path, capsys):
    path = tmp_path / "section.csv"
    run(["--type", "spur", "--csv", str(path)] + ANCHOR, capsys)
    zs = {line.split(",")[4] for line in path.read_text().strip().splitlines()[1:]}
    assert zs == {"0.000000"}
