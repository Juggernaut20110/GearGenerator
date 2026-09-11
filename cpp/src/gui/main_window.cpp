#include "gui/main_window.hpp"

#include "core/common/numerics.hpp"
#include "core/common/serialization.hpp"
#include "preview/export/export.hpp"
#include "preview/scene2d/bevel_scene.hpp"
#include "preview/scene2d/hypoid_scene.hpp"
#include "preview/scene2d/planetary_scene.hpp"
#include "preview/scene2d/spur_scene.hpp"
#include "preview/scene3d/assembly.hpp"
#include "solidworks/solidworks.hpp"

#include <QComboBox>
#include <QDir>
#include <QFileDialog>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMetaObject>
#include <QMessageBox>
#include <QPointer>
#include <QPushButton>
#include <QScrollArea>
#include <QSplitter>
#include <QSlider>
#include <QTableWidget>
#include <QTextEdit>
#include <QTimer>
#include <QToolBar>
#include <QThread>
#include <QVBoxLayout>
#include <QHeaderView>

#include <cmath>
#include <algorithm>
#include <filesystem>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <type_traits>

namespace geargen::gui {

namespace {

const FieldSpec* find_field(const char* key)
{
    for (const auto& field : MainWindow::all_fields()) {
        if (std::string(field.key) == key) return &field;
    }
    return nullptr;
}

QString number_text(double value)
{
    return QString::number(value, 'g', 12);
}

QString optional_number_text(const std::optional<double>& value)
{
    return value.has_value() ? number_text(*value) : QString{};
}

core::Hand hand_from_text(const std::string& text)
{
    if (text == "right") return core::Hand::Right;
    if (text == "left") return core::Hand::Left;
    throw std::invalid_argument("hand must be right or left");
}

} // namespace

const std::vector<FieldSpec>& MainWindow::all_fields()
{
    static const std::vector<FieldSpec> fields{
        {"module", "Module", "mm", FieldType::Number, {}},
        {"z1", "Pinion teeth z1", "", FieldType::Integer, {}},
        {"z2", "Gear teeth z2", "", FieldType::Integer, {}},
        {"z_sun", "Sun teeth", "", FieldType::Integer, {}},
        {"z_planet", "Planet teeth", "", FieldType::Integer, {}},
        {"n_planets", "Number of planets", "", FieldType::Integer, {}},
        {"internal", "Arrangement", "", FieldType::Choice, {"external", "internal"}},
        {"pressure_angle", "Pressure angle", "deg", FieldType::Number, {}},
        {"profile_shift_1", "Pinion profile shift x1", "", FieldType::Number, {}},
        {"profile_shift_2", "Gear/ring profile shift x2", "", FieldType::Number, {}},
        {"tip_alteration_mode", "Tip alteration mode", "", FieldType::Choice,
         {"iso_clearance", "legacy", "explicit"}},
        {"tip_alteration_coefficient", "Explicit tip alteration k", "",
         FieldType::OptionalNumber, {}},
        {"shaft_angle", "Shaft angle", "deg", FieldType::Number, {}},
        {"trace_kind", "Tooth trace", "", FieldType::DerivedChoice,
         {"straight", "zerol", "spiral"}},
        {"offset", "Hypoid offset", "mm", FieldType::Number, {}},
        {"spiral_angle", "Spiral angle", "deg", FieldType::Number, {}},
        {"helix_angle", "Helix angle", "deg", FieldType::Number, {}},
        {"hand", "Hand", "", FieldType::Choice, {"right", "left"}},
        {"cutter_radius", "Cutter radius", "mm", FieldType::OptionalNumber, {}},
        {"backlash", "Backlash", "mm", FieldType::Number, {}},
        {"backlash_mode", "Backlash definition", "", FieldType::Choice,
         {"working_circumferential", "normal", "legacy_reference"}},
        {"backlash_allocation", "Backlash to pinion", "", FieldType::Number, {}},
        {"face_width", "Face width", "mm", FieldType::Number, {}},
        {"bore", "Bore diameter", "mm", FieldType::Number, {}},
        {"hub_thickness", "Hub thickness", "mm", FieldType::Number, {}},
        {"rim_thickness", "Ring rim thickness", "mm", FieldType::Number, {}},
        {"min_root_thickness", "Min root thickness", "mm", FieldType::Number, {}},
        {"root_fillet_radius", "Root fillet radius", "mm", FieldType::OptionalNumber, {}},
    };
    return fields;
}

const std::vector<FieldSpec>& MainWindow::fields_for(const std::string& key)
{
    static const std::vector<FieldSpec> bevel{
        {"module", "Module (outer)", "mm", FieldType::Number, {}},
        {"z1", "Pinion teeth z1", "", FieldType::Integer, {}},
        {"z2", "Gear teeth z2", "", FieldType::Integer, {}},
        {"pressure_angle", "Pressure angle", "deg", FieldType::Number, {}},
        {"shaft_angle", "Shaft angle", "deg", FieldType::Number, {}},
        {"trace_kind", "Tooth trace", "", FieldType::DerivedChoice,
         {"straight", "zerol", "spiral"}},
        {"spiral_angle", "Spiral angle (mean)", "deg", FieldType::Number, {}},
        {"hand", "Hand (pinion)", "", FieldType::Choice, {"right", "left"}},
        {"cutter_radius", "Cutter radius", "mm", FieldType::OptionalNumber, {}},
        {"backlash", "Backlash", "mm", FieldType::Number, {}},
        {"face_width", "Face width", "mm", FieldType::Number, {}},
        {"bore", "Bore diameter", "mm", FieldType::Number, {}},
        {"hub_thickness", "Hub thickness", "mm", FieldType::Number, {}},
        {"min_root_thickness", "Min root thickness", "mm", FieldType::Number, {}},
    };
    static const std::vector<FieldSpec> spur{
        {"module", "Normal module", "mm", FieldType::Number, {}},
        {"z1", "Pinion teeth z1", "", FieldType::Integer, {}},
        {"z2", "Gear teeth z2", "", FieldType::Integer, {}},
        {"internal", "Arrangement", "", FieldType::Choice, {"external", "internal"}},
        {"pressure_angle", "Normal pressure angle", "deg", FieldType::Number, {}},
        {"profile_shift_1", "Pinion profile shift x1", "", FieldType::Number, {}},
        {"profile_shift_2", "Gear/ring profile shift x2", "", FieldType::Number, {}},
        {"tip_alteration_mode", "Tip alteration mode", "", FieldType::Choice,
         {"iso_clearance", "legacy", "explicit"}},
        {"tip_alteration_coefficient", "Explicit tip alteration k", "",
         FieldType::OptionalNumber, {}},
        {"helix_angle", "Helix angle", "deg", FieldType::Number, {}},
        {"hand", "Hand (pinion)", "", FieldType::Choice, {"right", "left"}},
        {"backlash", "Backlash", "mm", FieldType::Number, {}},
        {"backlash_mode", "Backlash definition", "", FieldType::Choice,
         {"working_circumferential", "normal", "legacy_reference"}},
        {"backlash_allocation", "Backlash to pinion", "", FieldType::Number, {}},
        {"face_width", "Face width", "mm", FieldType::Number, {}},
        {"bore", "Bore diameter", "mm", FieldType::Number, {}},
        {"hub_thickness", "Hub thickness", "mm", FieldType::Number, {}},
        {"rim_thickness", "Ring rim thickness", "mm", FieldType::Number, {}},
    };
    static const std::vector<FieldSpec> hypoid{
        {"module", "Outer transverse module", "mm", FieldType::Number, {}},
        {"z1", "Pinion teeth z1", "", FieldType::Integer, {}},
        {"z2", "Gear teeth z2", "", FieldType::Integer, {}},
        {"pressure_angle", "Pressure angle", "deg", FieldType::Number, {}},
        {"shaft_angle", "Shaft angle", "deg", FieldType::Number, {}},
        {"offset", "Hypoid offset", "mm", FieldType::Number, {}},
        {"spiral_angle", "Pinion spiral-angle magnitude", "deg", FieldType::Number, {}},
        {"hand", "Hand (pinion)", "", FieldType::Choice, {"right", "left"}},
        {"cutter_radius", "Cutter radius", "mm", FieldType::OptionalNumber, {}},
        {"backlash", "Outer transverse backlash", "mm", FieldType::Number, {}},
        {"face_width", "Face width", "mm", FieldType::Number, {}},
        {"bore", "Bore diameter", "mm", FieldType::Number, {}},
        {"hub_thickness", "Hub thickness", "mm", FieldType::Number, {}},
        {"min_root_thickness", "Min root thickness", "mm", FieldType::Number, {}},
        {"root_fillet_radius", "Root fillet radius", "mm", FieldType::OptionalNumber, {}},
    };
    static const std::vector<FieldSpec> planetary{
        {"module", "Normal module", "mm", FieldType::Number, {}},
        {"z_sun", "Sun teeth", "", FieldType::Integer, {}},
        {"z_planet", "Planet teeth", "", FieldType::Integer, {}},
        {"n_planets", "Number of planets", "", FieldType::Integer, {}},
        {"pressure_angle", "Normal pressure angle", "deg", FieldType::Number, {}},
        {"helix_angle", "Helix angle", "deg", FieldType::Number, {}},
        {"hand", "Hand (sun)", "", FieldType::Choice, {"right", "left"}},
        {"backlash", "Backlash", "mm", FieldType::Number, {}},
        {"face_width", "Face width", "mm", FieldType::Number, {}},
        {"bore", "Sun bore diameter", "mm", FieldType::Number, {}},
        {"hub_thickness", "Hub thickness", "mm", FieldType::Number, {}},
        {"rim_thickness", "Ring rim thickness", "mm", FieldType::Number, {}},
    };
    if (key == "bevel") return bevel;
    if (key == "spur") return spur;
    if (key == "hypoid") return hypoid;
    return planetary;
}

std::vector<std::pair<std::string, std::string>> MainWindow::scene_labels_for(
    const std::string& key)
{
    if (key == "bevel") return preview::bevel::scene_labels();
    if (key == "spur") return preview::spur::scene_labels();
    if (key == "hypoid") return preview::hypoid::scene_labels();
    return preview::planetary::scene_labels();
}

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent)
{
    setWindowTitle(QStringLiteral("Gear Generator - native Qt"));
    resize(1280, 820);
    create_widgets();
    params_ = default_params("bevel", 2.0, 17, 43);
    apply_kind(false);
    populate_from_params(params_);
    initialized_ = true;
    refresh_now();
}

void MainWindow::create_widgets()
{
    auto* central = new QWidget(this);
    auto* outer = new QHBoxLayout(central);
    outer->setContentsMargins(8, 8, 8, 8);
    auto* splitter = new QSplitter(Qt::Horizontal, central);
    outer->addWidget(splitter);

    auto* left = new QWidget(splitter);
    auto* left_layout = new QVBoxLayout(left);
    left_layout->setContentsMargins(0, 0, 6, 0);
    auto* type_row = new QHBoxLayout;
    type_row->addWidget(new QLabel(QStringLiteral("Gear type"), left));
    kind_combo_ = new QComboBox(left);
    kind_combo_->addItem(QStringLiteral("Bevel"), QStringLiteral("bevel"));
    kind_combo_->addItem(QStringLiteral("Spur"), QStringLiteral("spur"));
    kind_combo_->addItem(QStringLiteral("Hypoid"), QStringLiteral("hypoid"));
    kind_combo_->addItem(QStringLiteral("Planetary"), QStringLiteral("planetary"));
    type_row->addWidget(kind_combo_, 1);
    left_layout->addLayout(type_row);

    auto* scroll = new QScrollArea(left);
    scroll->setWidgetResizable(true);
    fields_panel_ = new QWidget(scroll);
    auto* fields_layout = new QVBoxLayout(fields_panel_);
    fields_layout->setContentsMargins(4, 6, 4, 6);
    for (const auto& field : all_fields()) {
        auto* row = new QWidget(fields_panel_);
        auto* row_layout = new QHBoxLayout(row);
        row_layout->setContentsMargins(0, 2, 0, 2);
        auto* label = new QLabel(QString::fromUtf8(field.label), row);
        label->setMinimumWidth(155);
        row_layout->addWidget(label);
        QWidget* control = nullptr;
        if (field.type == FieldType::Choice || field.type == FieldType::DerivedChoice) {
            auto* combo = new QComboBox(row);
            for (const auto& choice : field.choices)
                combo->addItem(QString::fromStdString(choice));
            combo->setMinimumWidth(142);
            combo->setProperty("field_key", QString::fromUtf8(field.key));
            control = combo;
            connect(combo, &QComboBox::currentTextChanged, this,
                    [this, key = std::string(field.key)] {
                        if (updating_widgets_) return;
                        if (key == "trace_kind") on_trace_changed();
                        else schedule_refresh();
                    });
        } else {
            auto* edit = new QLineEdit(row);
            edit->setAlignment(Qt::AlignRight);
            edit->setMinimumWidth(142);
            edit->setProperty("field_key", QString::fromUtf8(field.key));
            control = edit;
            connect(edit, &QLineEdit::textChanged, this,
                    [this] { if (!updating_widgets_) schedule_refresh(); });
        }
        row_layout->addWidget(control);
        auto* unit = new QLabel(QString::fromUtf8(field.unit), row);
        unit->setMinimumWidth(30);
        row_layout->addWidget(unit);
        fields_layout->addWidget(row);
        field_controls_.emplace(field.key, control);
        field_rows_.emplace(field.key, row);
    }
    fields_layout->addStretch(1);
    scroll->setWidget(fields_panel_);
    left_layout->addWidget(scroll, 1);

    auto* actions = new QHBoxLayout;
    auto* auto_button = new QPushButton(QStringLiteral("Auto-size blank"), left);
    actions->addWidget(auto_button);
    left_layout->addLayout(actions);
    connect(auto_button, &QPushButton::clicked, this, &MainWindow::auto_size);

    auto* preset_row = new QHBoxLayout;
    auto* load_button = new QPushButton(QStringLiteral("Load preset..."), left);
    auto* save_button = new QPushButton(QStringLiteral("Save preset..."), left);
    preset_row->addWidget(load_button);
    preset_row->addWidget(save_button);
    left_layout->addLayout(preset_row);
    connect(load_button, &QPushButton::clicked, this, &MainWindow::load_preset);
    connect(save_button, &QPushButton::clicked, this, &MainWindow::save_preset);

    auto* validation_box = new QGroupBox(QStringLiteral("Validation"), left);
    auto* validation_layout = new QVBoxLayout(validation_box);
    validation_text_ = new QTextEdit(validation_box);
    validation_text_->setReadOnly(true);
    validation_text_->setMinimumHeight(145);
    validation_layout->addWidget(validation_text_);
    left_layout->addWidget(validation_box);

    auto* right = new QWidget(splitter);
    auto* right_layout = new QVBoxLayout(right);
    right_layout->setContentsMargins(6, 0, 0, 0);
    auto* toolbar = new QHBoxLayout;
    toolbar->addWidget(new QLabel(QStringLiteral("Preview"), right));
    preview_mode_combo_ = new QComboBox(right);
    preview_mode_combo_->addItem(QStringLiteral("2D detail"), QStringLiteral("2d"));
    preview_mode_combo_->addItem(QStringLiteral("3D assembly"), QStringLiteral("3d"));
    toolbar->addWidget(preview_mode_combo_);
    member_label_ = new QLabel(QStringLiteral("Member"), right);
    toolbar->addWidget(member_label_);
    member_combo_ = new QComboBox(right);
    toolbar->addWidget(member_combo_);
    toolbar->addSpacing(10);
    scene_label_ = new QLabel(QStringLiteral("View"), right);
    toolbar->addWidget(scene_label_);
    scene_combo_ = new QComboBox(right);
    toolbar->addWidget(scene_combo_, 1);
    for (const auto& [name, text] : std::vector<std::pair<const char*, QString>>{
             {"front", QStringLiteral("Front")},
             {"top", QStringLiteral("Top")},
             {"right", QStringLiteral("Right")},
             {"iso", QStringLiteral("Iso")}}) {
        auto* button = new QPushButton(text, right);
        button->setProperty("camera_view", QString::fromUtf8(name));
        camera_buttons_.push_back(button);
        toolbar->addWidget(button);
        connect(button, &QPushButton::clicked, this,
                [this, name] { set_camera_view(name); });
    }
    auto* fit_button = new QPushButton(QStringLiteral("Fit"), right);
    toolbar->addWidget(fit_button);
    mesh_label_ = new QLabel(QStringLiteral("Mesh"), right);
    toolbar->addWidget(mesh_label_);
    mesh_slider_ = new QSlider(Qt::Horizontal, right);
    mesh_slider_->setRange(0, 1000);
    mesh_slider_->setValue(0);
    mesh_slider_->setMaximumWidth(110);
    mesh_slider_->setToolTip(QStringLiteral("Rotate the pinion through one tooth pitch"));
    toolbar->addWidget(mesh_slider_);
    auto* export_dxf_button = new QPushButton(QStringLiteral("Export DXF"), right);
    auto* export_csv_button = new QPushButton(QStringLiteral("Export CSV"), right);
    toolbar->addWidget(export_dxf_button);
    toolbar->addWidget(export_csv_button);
    build_button_ = new QPushButton(QStringLiteral("Build in SOLIDWORKS"), right);
    toolbar->addWidget(build_button_);
    right_layout->addLayout(toolbar);
    connect(member_combo_, &QComboBox::currentTextChanged, this,
            [this] { if (!updating_widgets_) { show_scene(); schedule_refresh(); } });
    connect(scene_combo_, &QComboBox::currentTextChanged, this,
            [this] { if (!updating_widgets_) show_scene(); });
    connect(preview_mode_combo_, &QComboBox::currentTextChanged, this,
            [this] { if (!updating_widgets_) on_preview_mode_changed(); });
    connect(mesh_slider_, &QSlider::valueChanged, this,
            [this](int value) { on_mesh_position_changed(value); });
    connect(fit_button, &QPushButton::clicked, this, &MainWindow::fit_preview);
    connect(export_dxf_button, &QPushButton::clicked, this, &MainWindow::export_dxf);
    connect(export_csv_button, &QPushButton::clicked, this, &MainWindow::export_csv);
    connect(build_button_, &QPushButton::clicked, this, &MainWindow::build_in_solidworks);

    scene_title_ = new QLabel(right);
    scene_title_->setStyleSheet(QStringLiteral("color:#444444;"));
    scene_title_->setWordWrap(true);
    right_layout->addWidget(scene_title_);
    preview_widget_ = new PreviewWidget(right);
    right_layout->addWidget(preview_widget_, 4);

    auto* derived_box = new QGroupBox(QStringLiteral("Derived values (read-only)"), right);
    auto* derived_layout = new QVBoxLayout(derived_box);
    derived_table_ = new QTableWidget(derived_box);
    derived_table_->setColumnCount(5);
    derived_table_->setHorizontalHeaderLabels({QStringLiteral("Quantity"), QStringLiteral("Pinion/Sun"),
                                                QStringLiteral("Gear/Planet"), QStringLiteral("Ring"),
                                                QStringLiteral("Unit")});
    derived_table_->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Stretch);
    for (int column = 1; column < 5; ++column)
        derived_table_->horizontalHeader()->setSectionResizeMode(column, QHeaderView::ResizeToContents);
    derived_table_->verticalHeader()->setVisible(false);
    derived_table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
    derived_layout->addWidget(derived_table_);
    right_layout->addWidget(derived_box, 2);

    status_label_ = new QLabel(right);
    status_label_->setWordWrap(true);
    status_label_->setStyleSheet(QStringLiteral("color:#444444;"));
    right_layout->addWidget(status_label_);

    splitter->addWidget(left);
    splitter->addWidget(right);
    splitter->setStretchFactor(0, 0);
    splitter->setStretchFactor(1, 1);
    setCentralWidget(central);

    debounce_timer_ = new QTimer(this);
    debounce_timer_->setSingleShot(true);
    debounce_timer_->setInterval(180);
    connect(debounce_timer_, &QTimer::timeout, this, &MainWindow::refresh_now);
    connect(kind_combo_, &QComboBox::currentTextChanged, this,
            [this] { if (!updating_widgets_) on_kind_changed(); });
    apply_preview_mode();
}

void MainWindow::apply_kind(bool)
{
    const auto active = fields_for(kind_key_);
    std::map<std::string, bool> visible;
    for (const auto& field : active) visible[field.key] = true;
    for (const auto& [key, row] : field_rows_) row->setVisible(visible.contains(key));

    updating_widgets_ = true;
    member_combo_->clear();
    if (kind_key_ == "planetary") {
        member_combo_->addItems({QStringLiteral("Sun"), QStringLiteral("Planet"), QStringLiteral("Ring")});
    } else {
        member_combo_->addItems({QStringLiteral("Pinion"), QStringLiteral("Gear")});
    }
    scene_combo_->clear();
    for (const auto& [key, label] : scene_labels_for(kind_key_)) {
        scene_combo_->addItem(QString::fromStdString(label), QString::fromStdString(key));
    }
    updating_widgets_ = false;
    if (scene_title_) scene_title_->clear();
}

void MainWindow::populate_from_params(const Params& params)
{
    updating_widgets_ = true;
    const auto set = [this](const char* key, const QString& value) { set_field_text(key, value); };
    if (const auto* p = std::get_if<core::BevelSetParams>(&params)) {
        set("module", number_text(p->module)); set("z1", QString::number(p->z1));
        set("z2", QString::number(p->z2)); set("pressure_angle", number_text(p->pressure_angle));
        set("shaft_angle", number_text(p->shaft_angle)); set("spiral_angle", number_text(p->spiral_angle));
        set("hand", QString::fromUtf8(core::hand_name(p->hand))); set("cutter_radius", optional_number_text(p->cutter_radius));
        set("backlash", number_text(p->backlash)); set("face_width", number_text(p->face_width));
        set("bore", number_text(p->bore)); set("hub_thickness", number_text(p->hub_thickness));
        set("min_root_thickness", number_text(p->min_root_thickness));
        set("trace_kind", p->is_curved() ? (std::abs(p->spiral_angle) > 1e-12 ? QStringLiteral("spiral") : QStringLiteral("zerol")) : QStringLiteral("straight"));
    } else if (const auto* p = std::get_if<core::SpurSetParams>(&params)) {
        set("module", number_text(p->module)); set("z1", QString::number(p->z1)); set("z2", QString::number(p->z2));
        set("internal", p->internal ? QStringLiteral("internal") : QStringLiteral("external"));
        set("pressure_angle", number_text(p->pressure_angle)); set("profile_shift_1", number_text(p->profile_shift_1));
        set("profile_shift_2", number_text(p->profile_shift_2)); set("tip_alteration_mode", QString::fromStdString(p->tip_alteration_mode));
        set("tip_alteration_coefficient", optional_number_text(p->tip_alteration_coefficient));
        set("helix_angle", number_text(p->helix_angle)); set("hand", QString::fromUtf8(core::hand_name(p->hand)));
        set("backlash", number_text(p->backlash)); set("backlash_mode", QString::fromStdString(p->backlash_mode));
        set("backlash_allocation", number_text(p->backlash_allocation)); set("face_width", number_text(p->face_width));
        set("bore", number_text(p->bore)); set("hub_thickness", number_text(p->hub_thickness));
        set("rim_thickness", number_text(p->rim_thickness));
    } else if (const auto* p = std::get_if<core::HypoidSetParams>(&params)) {
        set("module", number_text(p->module)); set("z1", QString::number(p->z1)); set("z2", QString::number(p->z2));
        set("pressure_angle", number_text(p->pressure_angle)); set("shaft_angle", number_text(p->shaft_angle));
        set("offset", number_text(p->offset)); set("spiral_angle", number_text(p->spiral_angle));
        set("hand", QString::fromUtf8(core::hand_name(p->hand))); set("cutter_radius", optional_number_text(p->cutter_radius));
        set("backlash", number_text(p->backlash)); set("face_width", number_text(p->face_width));
        set("bore", number_text(p->bore)); set("hub_thickness", number_text(p->hub_thickness));
        set("min_root_thickness", number_text(p->min_root_thickness)); set("root_fillet_radius", optional_number_text(p->root_fillet_radius));
    } else if (const auto* p = std::get_if<core::PlanetarySetParams>(&params)) {
        set("module", number_text(p->module)); set("z_sun", QString::number(p->z_sun)); set("z_planet", QString::number(p->z_planet));
        set("n_planets", QString::number(p->n_planets)); set("pressure_angle", number_text(p->pressure_angle));
        set("helix_angle", number_text(p->helix_angle)); set("hand", QString::fromUtf8(core::hand_name(p->hand)));
        set("backlash", number_text(p->backlash)); set("face_width", number_text(p->face_width));
        set("bore", number_text(p->bore)); set("hub_thickness", number_text(p->hub_thickness)); set("rim_thickness", number_text(p->rim_thickness));
    }
    updating_widgets_ = false;
}

QString MainWindow::field_text(const char* key) const
{
    const auto found = field_controls_.find(key);
    if (found == field_controls_.end()) return {};
    if (const auto* edit = qobject_cast<const QLineEdit*>(found->second)) return edit->text();
    if (const auto* combo = qobject_cast<const QComboBox*>(found->second)) return combo->currentText();
    return {};
}

void MainWindow::set_field_text(const char* key, const QString& text)
{
    const auto found = field_controls_.find(key);
    if (found == field_controls_.end()) return;
    if (auto* edit = qobject_cast<QLineEdit*>(found->second)) edit->setText(text);
    else if (auto* combo = qobject_cast<QComboBox*>(found->second)) {
        const int index = combo->findText(text);
        if (index >= 0) combo->setCurrentIndex(index);
    }
}

std::optional<double> MainWindow::read_optional_double(const FieldSpec& field,
                                                       std::vector<std::string>& errors) const
{
    const QString text = field_text(field.key).trimmed();
    if (text.isEmpty()) return std::nullopt;
    return read_double(field, errors);
}

std::optional<double> MainWindow::read_double(const FieldSpec& field,
                                              std::vector<std::string>& errors) const
{
    bool ok = false;
    const QString text = field_text(field.key).trimmed();
    const double value = text.toDouble(&ok);
    if (!ok || !std::isfinite(value)) {
        errors.push_back(std::string(field.key) + " — " + field.label + ": must be a finite number");
        return std::nullopt;
    }
    return value;
}

std::optional<int> MainWindow::read_int(const FieldSpec& field,
                                        std::vector<std::string>& errors) const
{
    const auto value = read_double(field, errors);
    if (!value.has_value()) return std::nullopt;
    const double rounded = std::round(*value);
    if (std::abs(*value - rounded) > 1e-9 || rounded < static_cast<double>(std::numeric_limits<int>::min()) ||
        rounded > static_cast<double>(std::numeric_limits<int>::max())) {
        errors.push_back(std::string(field.key) + " — " + field.label + ": must be a whole number");
        return std::nullopt;
    }
    return static_cast<int>(rounded);
}

std::optional<std::string> MainWindow::read_choice(const FieldSpec& field,
                                                   std::vector<std::string>& errors) const
{
    const std::string value = field_text(field.key).trimmed().toStdString();
    for (const auto& choice : field.choices) if (value == choice) return value;
    errors.push_back(std::string(field.key) + " — " + field.label + ": must be one of the listed choices");
    return std::nullopt;
}

MainWindow::Params MainWindow::parse_params(std::vector<std::string>& errors) const
{
    const auto field = [](const char* key) -> const FieldSpec& {
        const auto* result = find_field(key);
        if (result == nullptr) throw std::logic_error("missing GUI field metadata");
        return *result;
    };
    const auto module_value = read_double(field("module"), errors);
    const auto first_value = read_int(field(kind_key_ == "planetary" ? "z_sun" : "z1"), errors);
    const auto second_value = read_int(field(kind_key_ == "planetary" ? "z_planet" : "z2"), errors);
    if (!module_value || !first_value || !second_value) return {};

    if (kind_key_ == "bevel") {
        core::BevelDefaultOverrides o;
        o.pressure_angle = read_double(field("pressure_angle"), errors);
        o.shaft_angle = read_double(field("shaft_angle"), errors);
        o.spiral_angle = read_double(field("spiral_angle"), errors);
        if (const auto value = read_choice(field("hand"), errors)) o.hand = hand_from_text(*value);
        o.cutter_radius = read_optional_double(field("cutter_radius"), errors);
        o.backlash = read_double(field("backlash"), errors);
        o.face_width = read_double(field("face_width"), errors);
        o.bore = read_double(field("bore"), errors);
        o.hub_thickness = read_double(field("hub_thickness"), errors);
        o.min_root_thickness = read_double(field("min_root_thickness"), errors);
        if (!errors.empty()) return {};
        return core::BevelSetParams::with_defaults(*module_value, *first_value, *second_value, false, o);
    }
    if (kind_key_ == "spur") {
        core::SpurDefaultOverrides o;
        if (const auto value = read_choice(field("internal"), errors)) o.internal = *value == "internal";
        o.pressure_angle = read_double(field("pressure_angle"), errors);
        o.profile_shift_1 = read_double(field("profile_shift_1"), errors);
        o.profile_shift_2 = read_double(field("profile_shift_2"), errors);
        o.tip_alteration_mode = read_choice(field("tip_alteration_mode"), errors);
        o.tip_alteration_coefficient = read_optional_double(field("tip_alteration_coefficient"), errors);
        o.helix_angle = read_double(field("helix_angle"), errors);
        if (const auto value = read_choice(field("hand"), errors)) o.hand = hand_from_text(*value);
        o.backlash = read_double(field("backlash"), errors);
        o.backlash_mode = read_choice(field("backlash_mode"), errors);
        o.backlash_allocation = read_double(field("backlash_allocation"), errors);
        o.face_width = read_double(field("face_width"), errors);
        o.bore = read_double(field("bore"), errors);
        o.hub_thickness = read_double(field("hub_thickness"), errors);
        o.rim_thickness = read_double(field("rim_thickness"), errors);
        if (!errors.empty()) return {};
        return core::SpurSetParams::with_defaults(*module_value, *first_value, *second_value, o);
    }
    if (kind_key_ == "hypoid") {
        core::HypoidDefaultOverrides o;
        o.pressure_angle = read_double(field("pressure_angle"), errors);
        o.shaft_angle = read_double(field("shaft_angle"), errors);
        o.offset = read_double(field("offset"), errors);
        o.spiral_angle = read_double(field("spiral_angle"), errors);
        if (const auto value = read_choice(field("hand"), errors)) o.hand = hand_from_text(*value);
        o.cutter_radius = read_optional_double(field("cutter_radius"), errors);
        o.backlash = read_double(field("backlash"), errors);
        o.face_width = read_double(field("face_width"), errors);
        o.bore = read_double(field("bore"), errors);
        o.hub_thickness = read_double(field("hub_thickness"), errors);
        o.min_root_thickness = read_double(field("min_root_thickness"), errors);
        o.root_fillet_radius = read_optional_double(field("root_fillet_radius"), errors);
        if (!errors.empty()) return {};
        return core::HypoidSetParams::with_defaults(*module_value, *first_value, *second_value, o);
    }
    core::PlanetaryDefaultOverrides o;
    o.n_planets = read_int(field("n_planets"), errors);
    o.pressure_angle = read_double(field("pressure_angle"), errors);
    o.helix_angle = read_double(field("helix_angle"), errors);
    if (const auto value = read_choice(field("hand"), errors)) o.hand = hand_from_text(*value);
    o.backlash = read_double(field("backlash"), errors);
    o.face_width = read_double(field("face_width"), errors);
    o.bore = read_double(field("bore"), errors);
    o.hub_thickness = read_double(field("hub_thickness"), errors);
    o.rim_thickness = read_double(field("rim_thickness"), errors);
    if (!errors.empty()) return {};
    return core::PlanetarySetParams::with_defaults(*module_value, *first_value, *second_value, o);
}

MainWindow::Params MainWindow::default_params(const std::string& key, double module_value,
                                              int first, int second) const
{
    if (key == "bevel") return core::BevelSetParams::with_defaults(module_value, first, second);
    if (key == "spur") {
        core::SpurDefaultOverrides o;
        o.backlash_mode = std::string("working_circumferential");
        return core::SpurSetParams::with_defaults(module_value, first, second, o);
    }
    if (key == "hypoid") return core::HypoidSetParams::with_defaults(module_value, first, second);
    return core::PlanetarySetParams::with_defaults(module_value, first, second);
}

std::pair<int, int> MainWindow::counts(const Params& params) const
{
    if (const auto* p = std::get_if<core::BevelSetParams>(&params)) return {p->z1, p->z2};
    if (const auto* p = std::get_if<core::SpurSetParams>(&params)) return {p->z1, p->z2};
    if (const auto* p = std::get_if<core::HypoidSetParams>(&params)) return {p->z1, p->z2};
    if (const auto* p = std::get_if<core::PlanetarySetParams>(&params)) return {p->z_sun, p->z_planet};
    return {17, 43};
}

double MainWindow::module(const Params& params) const
{
    return std::visit([](const auto& value) -> double {
        using T = std::decay_t<decltype(value)>;
        if constexpr (std::is_same_v<T, std::monostate>) return 2.0;
        else return value.module;
    }, params);
}

std::string MainWindow::member_name() const
{
    return member_combo_->currentText().toLower().toStdString();
}

void MainWindow::schedule_refresh()
{
    if (debounce_timer_ && initialized_) debounce_timer_->start();
}

void MainWindow::refresh_now()
{
    std::vector<std::string> parse_errors;
    const Params parsed = parse_params(parse_errors);
    if (!parse_errors.empty() || std::holds_alternative<std::monostate>(parsed)) {
        has_geometry_ = false;
        geometry_ = {};
        validation_ = {};
        preview_widget_->clear_scene();
        derived_table_->setRowCount(0);
        show_validation(parse_errors);
        build_button_->setEnabled(false);
        set_status(QStringLiteral("input incomplete or invalid"));
        return;
    }
    params_ = parsed;
    validation_ = std::visit([](const auto& value) -> core::ValidationResult {
        using T = std::decay_t<decltype(value)>;
        if constexpr (std::is_same_v<T, core::BevelSetParams>) return core::validate(value);
        if constexpr (std::is_same_v<T, core::SpurSetParams>) return core::validate(value);
        if constexpr (std::is_same_v<T, core::HypoidSetParams>) return core::validate(value);
        if constexpr (std::is_same_v<T, core::PlanetarySetParams>) return core::validate(value);
        return {};
    }, params_);
    show_validation({});
    if (!validation_.ok()) {
        has_geometry_ = false;
        geometry_ = {};
        preview_widget_->clear_scene();
        derived_table_->setRowCount(0);
        build_button_->setEnabled(false);
        set_status(QStringLiteral("build blocked by validation errors"));
        return;
    }
    try {
        geometry_ = std::visit([](const auto& value) -> Geometry {
            using T = std::decay_t<decltype(value)>;
            if constexpr (std::is_same_v<T, core::BevelSetParams>) return core::bevel::derive(value);
            if constexpr (std::is_same_v<T, core::SpurSetParams>) return core::spur::derive(value);
            if constexpr (std::is_same_v<T, core::HypoidSetParams>) return core::hypoid::derive(value);
            if constexpr (std::is_same_v<T, core::PlanetarySetParams>) return core::planetary::derive(value);
            return {};
        }, params_);
        has_geometry_ = true;
        show_derived();
        show_scene();
        build_button_->setEnabled(!solidworks_build_running_);
        set_status(QStringLiteral("ready - geometry valid"));
    } catch (const std::exception& error) {
        has_geometry_ = false;
        geometry_ = {};
        preview_widget_->clear_scene();
        build_button_->setEnabled(false);
        show_validation({std::string("geometry — ") + error.what()});
        set_status(QStringLiteral("geometry calculation failed"));
    }
}

void MainWindow::show_validation(const std::vector<std::string>& parse_errors)
{
    for (const auto& [key, widget] : field_controls_) {
        if (auto* edit = qobject_cast<QLineEdit*>(widget)) edit->setStyleSheet({});
        else if (auto* combo = qobject_cast<QComboBox*>(widget)) combo->setStyleSheet({});
    }
    QString html;
    const auto add = [&html](const QString& heading, const QString& colour,
                             const std::vector<std::string>& issues) {
        if (issues.empty()) return;
        html += QStringLiteral("<b>%1</b><br>").arg(heading);
        for (const auto& issue : issues)
            html += QStringLiteral("<span style='color:%1'>%2</span><br>")
                        .arg(colour, QString::fromStdString(issue).toHtmlEscaped());
        html += QStringLiteral("<br>");
    };
    add(QStringLiteral("Input errors"), QStringLiteral("#b0202a"), parse_errors);
    std::vector<std::string> errors;
    for (const auto& issue : validation_.errors) errors.push_back(issue.field + " — " + issue.message);
    std::vector<std::string> warnings;
    for (const auto& issue : validation_.warnings) warnings.push_back(issue.field + " — " + issue.message);
    std::vector<std::string> advisories;
    for (const auto& issue : validation_.advisories) advisories.push_back(issue.field + " — " + issue.message);
    add(QStringLiteral("Errors"), QStringLiteral("#b0202a"), errors);
    add(QStringLiteral("Warnings"), QStringLiteral("#8a6100"), warnings);
    add(QStringLiteral("Advisory"), QStringLiteral("#555555"), advisories);
    if (html.isEmpty()) html = QStringLiteral("<span style='color:#1f7a3d'>No validation errors.</span>");
    validation_text_->setHtml(html);

    auto mark = [this](const std::vector<std::string>& issues) {
        for (const auto& issue : issues) {
            const auto separator = issue.find(" — ");
            if (separator == std::string::npos) continue;
            const auto found = field_controls_.find(issue.substr(0, separator));
            if (found == field_controls_.end()) continue;
            found->second->setStyleSheet(QStringLiteral("background:#ffe1e1;"));
        }
    };
    mark(parse_errors); mark(errors);
}

void MainWindow::show_derived()
{
    derived_table_->setRowCount(0);
    if (!has_geometry_) return;
    std::vector<preview::Row> rows;
    if (const auto* g = std::get_if<core::bevel::SetGeometry>(&geometry_)) rows = preview::bevel::derived_rows(*g);
    else if (const auto* g = std::get_if<core::spur::SetGeometry>(&geometry_)) rows = preview::spur::derived_rows(*g);
    else if (const auto* g = std::get_if<core::hypoid::SetGeometry>(&geometry_)) rows = preview::hypoid::derived_rows(*g);
    else if (const auto* g = std::get_if<core::planetary::SetGeometry>(&geometry_)) rows = preview::planetary::derived_rows(*g);
    derived_table_->setRowCount(static_cast<int>(rows.size()));
    for (int row = 0; row < static_cast<int>(rows.size()); ++row) {
        const auto& value = rows[static_cast<std::size_t>(row)];
        const QStringList columns{QString::fromStdString(value.label), QString::fromStdString(value.pinion),
                                  QString::fromStdString(value.gear), QString::fromStdString(value.third),
                                  QString::fromStdString(value.unit)};
        for (int column = 0; column < columns.size(); ++column) {
            auto* item = new QTableWidgetItem(columns[column]);
            if (value.header) {
                item->setBackground(QColor(QStringLiteral("#eaeff4")));
                QFont font = item->font(); font.setBold(true); item->setFont(font);
            }
            derived_table_->setItem(row, column, item);
        }
    }
}

void MainWindow::show_scene()
{
    if (!has_geometry_) return;
    const std::string member = member_name();
    const std::string key = scene_combo_->currentData().toString().toStdString();
    const bool three_d = preview_mode_combo_->currentData().toString() == QStringLiteral("3d");

    try {
        if (!key.empty()) {
            if (const auto* g = std::get_if<core::bevel::SetGeometry>(&geometry_))
                scene_ = preview::bevel::build_scene(*g, member, key);
            else if (const auto* g = std::get_if<core::spur::SetGeometry>(&geometry_))
                scene_ = preview::spur::build_scene(*g, member, key);
            else if (const auto* g = std::get_if<core::hypoid::SetGeometry>(&geometry_))
                scene_ = preview::hypoid::build_scene(*g, member, key);
            else if (const auto* g = std::get_if<core::planetary::SetGeometry>(&geometry_))
                scene_ = preview::planetary::build_scene(*g, member, key);
        }

        if (!three_d) {
            preview_widget_->set_scene(scene_);
            scene_title_->setText(QString::fromStdString(scene_.title));
            return;
        }

        const auto [first, unused_second] = counts(params_);
        (void)unused_second;
        const double mesh_position = kind_key_ == "planetary"
            ? 0.0
            : static_cast<double>(mesh_slider_->value()) / 1000.0 *
                  core::kTau / static_cast<double>(std::max(first, 1));
        if (const auto* g = std::get_if<core::bevel::SetGeometry>(&geometry_))
            scene3d_ = preview::scene3d::build_scene(*g, mesh_position);
        else if (const auto* g = std::get_if<core::spur::SetGeometry>(&geometry_))
            scene3d_ = preview::scene3d::build_scene(*g, mesh_position);
        else if (const auto* g = std::get_if<core::hypoid::SetGeometry>(&geometry_))
            scene3d_ = preview::scene3d::build_scene(*g, mesh_position);
        else if (const auto* g = std::get_if<core::planetary::SetGeometry>(&geometry_))
            scene3d_ = preview::scene3d::build_scene(*g, mesh_position);
        else
            return;
        preview_widget_->set_scene3d(scene3d_);
        scene_title_->setText(QString::fromStdString(scene3d_.title));
    } catch (const std::exception& error) {
        preview_widget_->clear_scene();
        scene_title_->setText(QStringLiteral("Preview unavailable: %1")
                                  .arg(QString::fromUtf8(error.what())));
    }
}

void MainWindow::apply_preview_mode()
{
    if (preview_mode_combo_ == nullptr) return;
    const bool three_d = preview_mode_combo_->currentData().toString() == QStringLiteral("3d");
    member_label_->setVisible(!three_d);
    member_combo_->setVisible(!three_d);
    scene_label_->setVisible(!three_d);
    scene_combo_->setVisible(!three_d);
    mesh_label_->setVisible(three_d);
    mesh_slider_->setVisible(three_d);
    for (auto* button : camera_buttons_) button->setVisible(three_d);
    mesh_slider_->setEnabled(three_d && kind_key_ != "planetary");
}

void MainWindow::on_preview_mode_changed()
{
    apply_preview_mode();
    show_scene();
    if (preview_mode_combo_->currentData().toString() == QStringLiteral("3d"))
        preview_widget_->set_camera_view("iso");
    fit_preview();
}

void MainWindow::on_mesh_position_changed(int)
{
    if (!initialized_ || !has_geometry_ ||
        preview_mode_combo_->currentData().toString() != QStringLiteral("3d")) return;
    show_scene();
}

void MainWindow::set_camera_view(const char* name)
{
    if (preview_mode_combo_->currentData().toString() != QStringLiteral("3d")) return;
    preview_widget_->set_camera_view(name);
}

void MainWindow::set_status(const QString& text)
{
    QString detail = text;
    if (has_geometry_) {
        if (const auto* g = std::get_if<core::spur::SetGeometry>(&geometry_))
            detail += QStringLiteral("   m_n %1   %2:%3   a_w %4 mm")
                .arg(g->parameters.module).arg(g->parameters.z1).arg(g->parameters.z2).arg(g->working_centre_distance_mm, 0, 'f', 3);
        else if (const auto* g = std::get_if<core::bevel::SetGeometry>(&geometry_))
            detail += QStringLiteral("   m %1   %2:%3   cones %4 / %5 deg")
                .arg(g->parameters.module).arg(g->parameters.z1).arg(g->parameters.z2)
                .arg(g->pinion.pitch_angle_deg(), 0, 'f', 3).arg(g->gear.pitch_angle_deg(), 0, 'f', 3);
        else if (const auto* g = std::get_if<core::hypoid::SetGeometry>(&geometry_))
            detail += QStringLiteral("   m_o %1   %2:%3   offset %4 mm")
                .arg(g->parameters.module).arg(g->parameters.z1).arg(g->parameters.z2).arg(g->parameters.offset);
        else if (const auto* g = std::get_if<core::planetary::SetGeometry>(&geometry_))
            detail += QStringLiteral("   sun %1 / %2 planets of %3 / ring %4   a %5 mm")
                .arg(g->parameters.z_sun).arg(g->parameters.n_planets).arg(g->parameters.z_planet)
                .arg(g->parameters.z_ring()).arg(g->centre_distance_mm, 0, 'f', 3);
    }
    status_label_->setText(detail);
}

void MainWindow::on_kind_changed()
{
    const auto old_params = params_;
    const auto old_counts = counts(old_params);
    const double old_module = module(old_params);
    kind_key_ = kind_combo_->currentData().toString().toStdString();
    apply_kind(true);
    apply_preview_mode();
    params_ = default_params(kind_key_, old_module, old_counts.first, old_counts.second);
    populate_from_params(params_);
    refresh_now();
}

void MainWindow::on_trace_changed()
{
    if (kind_key_ != "bevel") return;
    std::vector<std::string> errors;
    const auto parsed = parse_params(errors);
    const auto* p = std::get_if<core::BevelSetParams>(&parsed);
    if (p == nullptr || !errors.empty()) return;
    const QString trace = field_text("trace_kind");
    double spiral = 0.0;
    std::optional<double> cutter = p->cutter_radius;
    if (trace == QStringLiteral("spiral")) {
        spiral = std::abs(p->spiral_angle) > 1e-12 ? p->spiral_angle : 35.0;
        if (!cutter) {
            core::BevelDefaultOverrides o; o.shaft_angle = p->shaft_angle;
            cutter = core::BevelSetParams::with_defaults(p->module, p->z1, p->z2, true, o).cutter_radius;
        }
    } else if (trace == QStringLiteral("zerol")) {
        core::BevelDefaultOverrides o; o.shaft_angle = p->shaft_angle;
        if (!cutter) cutter = core::BevelSetParams::with_defaults(p->module, p->z1, p->z2, true, o).cutter_radius;
    }
    updating_widgets_ = true;
    set_field_text("spiral_angle", number_text(spiral));
    set_field_text("cutter_radius", optional_number_text(cutter));
    updating_widgets_ = false;
    refresh_now();
}

void MainWindow::auto_size()
{
    std::vector<std::string> errors;
    const auto parsed = parse_params(errors);
    if (!errors.empty() || std::holds_alternative<std::monostate>(parsed)) {
        show_validation(errors); return;
    }
    const auto [first, second] = counts(parsed);
    const double module_value = module(parsed);
    if (kind_key_ == "bevel") {
        const auto& p = std::get<core::BevelSetParams>(parsed);
        core::BevelDefaultOverrides o; o.pressure_angle = p.pressure_angle; o.shaft_angle = p.shaft_angle;
        o.spiral_angle = p.spiral_angle; o.hand = p.hand; o.cutter_radius = p.cutter_radius;
        const auto sized = core::BevelSetParams::with_defaults(module_value, first, second, false, o);
        updating_widgets_ = true; set_field_text("face_width", number_text(sized.face_width)); set_field_text("bore", number_text(sized.bore));
        set_field_text("hub_thickness", number_text(sized.hub_thickness)); set_field_text("min_root_thickness", number_text(sized.min_root_thickness)); updating_widgets_ = false;
    } else if (kind_key_ == "spur") {
        const auto& p = std::get<core::SpurSetParams>(parsed);
        core::SpurDefaultOverrides o; o.pressure_angle = p.pressure_angle; o.helix_angle = p.helix_angle; o.hand = p.hand; o.internal = p.internal;
        const auto sized = core::SpurSetParams::with_defaults(module_value, first, second, o);
        updating_widgets_ = true; set_field_text("face_width", number_text(sized.face_width)); set_field_text("bore", number_text(sized.bore));
        set_field_text("hub_thickness", number_text(sized.hub_thickness)); set_field_text("rim_thickness", number_text(sized.rim_thickness)); updating_widgets_ = false;
    } else if (kind_key_ == "hypoid") {
        const auto& p = std::get<core::HypoidSetParams>(parsed);
        core::HypoidDefaultOverrides o; o.pressure_angle = p.pressure_angle; o.shaft_angle = p.shaft_angle; o.offset = p.offset;
        o.spiral_angle = p.spiral_angle; o.hand = p.hand; o.cutter_radius = p.cutter_radius;
        const auto sized = core::HypoidSetParams::with_defaults(module_value, first, second, o);
        updating_widgets_ = true; set_field_text("face_width", number_text(sized.face_width)); set_field_text("bore", number_text(sized.bore));
        set_field_text("hub_thickness", number_text(sized.hub_thickness)); set_field_text("min_root_thickness", number_text(sized.min_root_thickness)); updating_widgets_ = false;
    } else {
        const auto& p = std::get<core::PlanetarySetParams>(parsed);
        core::PlanetaryDefaultOverrides o; o.n_planets = p.n_planets; o.pressure_angle = p.pressure_angle; o.helix_angle = p.helix_angle; o.hand = p.hand;
        const auto sized = core::PlanetarySetParams::with_defaults(module_value, first, second, o);
        updating_widgets_ = true; set_field_text("face_width", number_text(sized.face_width)); set_field_text("bore", number_text(sized.bore));
        set_field_text("hub_thickness", number_text(sized.hub_thickness)); set_field_text("rim_thickness", number_text(sized.rim_thickness)); updating_widgets_ = false;
    }
    refresh_now();
}

void MainWindow::fit_preview() { preview_widget_->fit_view(); }

void MainWindow::export_dxf()
{
    if (!has_geometry_) return;
    const QString path = QFileDialog::getSaveFileName(this, QStringLiteral("Export DXF"), {}, QStringLiteral("DXF (*.dxf)"));
    if (path.isEmpty()) return;
    try {
        preview::exporter::write_dxf(std::filesystem::path(path.toStdString()), scene_);
        set_status(QStringLiteral("DXF exported: %1").arg(path));
    } catch (const std::exception& error) {
        QMessageBox::critical(this, QStringLiteral("Export failed"), QString::fromUtf8(error.what()));
    }
}

void MainWindow::export_csv()
{
    if (!has_geometry_) return;
    const QString path = QFileDialog::getSaveFileName(this, QStringLiteral("Export CSV"), {}, QStringLiteral("CSV (*.csv)"));
    if (path.isEmpty()) return;
    try {
        const auto output = std::filesystem::path(path.toStdString());
        if (const auto* g = std::get_if<core::bevel::SetGeometry>(&geometry_)) preview::bevel::write_csv(output, *g, member_name());
        else if (const auto* g = std::get_if<core::spur::SetGeometry>(&geometry_)) preview::spur::write_csv(output, *g, member_name());
        else if (const auto* g = std::get_if<core::hypoid::SetGeometry>(&geometry_)) preview::hypoid::write_csv(output, *g, member_name());
        else if (const auto* g = std::get_if<core::planetary::SetGeometry>(&geometry_)) preview::planetary::write_csv(output, *g, member_name());
        set_status(QStringLiteral("CSV exported: %1").arg(path));
    } catch (const std::exception& error) {
        QMessageBox::critical(this, QStringLiteral("Export failed"), QString::fromUtf8(error.what()));
    }
}

void MainWindow::load_preset()
{
    const QString path = QFileDialog::getOpenFileName(this, QStringLiteral("Load preset"), {}, QStringLiteral("JSON (*.json)"));
    if (path.isEmpty()) return;
    try {
        const auto file = std::filesystem::path(path.toStdString());
        if (kind_key_ == "bevel") params_ = core::load_bevel_preset(file);
        else if (kind_key_ == "spur") params_ = core::load_spur_preset(file);
        else if (kind_key_ == "hypoid") params_ = core::load_hypoid_preset(file);
        else params_ = core::load_planetary_preset(file);
        populate_from_params(params_);
        refresh_now();
    } catch (const std::exception& error) {
        QMessageBox::critical(this, QStringLiteral("Preset load failed"), QString::fromUtf8(error.what()));
    }
}

void MainWindow::save_preset()
{
    if (std::holds_alternative<std::monostate>(params_)) return;
    const QString path = QFileDialog::getSaveFileName(this, QStringLiteral("Save preset"), {}, QStringLiteral("JSON (*.json)"));
    if (path.isEmpty()) return;
    try {
        const auto file = std::filesystem::path(path.toStdString());
        if (const auto* p = std::get_if<core::BevelSetParams>(&params_)) core::save_preset(file, *p);
        else if (const auto* p = std::get_if<core::SpurSetParams>(&params_)) core::save_preset(file, *p);
        else if (const auto* p = std::get_if<core::HypoidSetParams>(&params_)) core::save_preset(file, *p);
        else if (const auto* p = std::get_if<core::PlanetarySetParams>(&params_)) core::save_preset(file, *p);
        set_status(QStringLiteral("Preset saved: %1").arg(path));
    } catch (const std::exception& error) {
        QMessageBox::critical(this, QStringLiteral("Preset save failed"), QString::fromUtf8(error.what()));
    }
}

void MainWindow::build_in_solidworks()
{
    if (!has_geometry_ || !validation_.ok() || solidworks_build_running_) return;
    const QString directory = QFileDialog::getExistingDirectory(
        this, QStringLiteral("SOLIDWORKS output directory"), QDir::currentPath());
    if (directory.isEmpty()) return;

    solidworks::Parameters build_parameters;
    if (const auto* value = std::get_if<core::BevelSetParams>(&params_))
        build_parameters = *value;
    else if (const auto* value = std::get_if<core::SpurSetParams>(&params_))
        build_parameters = *value;
    else if (const auto* value = std::get_if<core::HypoidSetParams>(&params_))
        build_parameters = *value;
    else if (const auto* value = std::get_if<core::PlanetarySetParams>(&params_))
        build_parameters = *value;
    else
        return;

    solidworks::BuildRequest request;
    request.parameters = std::move(build_parameters);
    request.output_directory = std::filesystem::path(directory.toStdString());
    request.options.visible = true;
    request.options.silent = true;
    request.options.save_assembly = true;
    request.options.mate = true;

    solidworks_build_running_ = true;
    build_button_->setEnabled(false);
    set_status(QStringLiteral("building in SOLIDWORKS..."));

    QPointer<MainWindow> self(this);
    auto* worker = QThread::create([self, request = std::move(request)]() mutable {
        // solidworks::build owns the Session, and therefore initializes and
        // releases COM on this worker thread. No COM wrapper crosses back to Qt.
        auto result = solidworks::build(request);
        if (!self) return;
        QMetaObject::invokeMethod(self,
            [self, result = std::move(result)]() mutable {
                if (!self) return;
                self->solidworks_build_running_ = false;
                self->build_button_->setEnabled(self->has_geometry_ && self->validation_.ok());
                if (result.ok) {
                    self->set_status(QString::fromStdString(result.message));
                    QMessageBox::information(self, QStringLiteral("SOLIDWORKS build"),
                        QString::fromStdString(result.message +
                            (result.assembly_path.empty()
                                 ? std::string{}
                                 : "\nSaved: " + result.assembly_path.string())));
                } else {
                    self->set_status(QStringLiteral("SOLIDWORKS build failed"));
                    QMessageBox::critical(self, QStringLiteral("SOLIDWORKS build failed"),
                                           QString::fromStdString(result.message));
                }
            }, Qt::QueuedConnection);
    });
    connect(worker, &QThread::finished, worker, &QObject::deleteLater);
    worker->start();
}

} // namespace geargen::gui
