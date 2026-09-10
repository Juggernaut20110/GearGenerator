#pragma once

#include "core/bevel/bevel.hpp"
#include "core/common/parameters.hpp"
#include "core/hypoid/hypoid.hpp"
#include "core/planetary/planetary.hpp"
#include "core/spur/spur.hpp"
#include "core/validation/validation.hpp"
#include "preview/scene2d/scene.hpp"
#include "gui/preview_widget.hpp"

#include <QMainWindow>

#include <map>
#include <optional>
#include <string>
#include <variant>
#include <vector>

class QComboBox;
class QLabel;
class QLineEdit;
class QPushButton;
class QTableWidget;
class QTextEdit;
class QTimer;

namespace geargen::gui {

enum class FieldType { Number, Integer, Choice, OptionalNumber, DerivedChoice };

struct FieldSpec {
    const char* key;
    const char* label;
    const char* unit;
    FieldType type;
    std::vector<std::string> choices;
};

class MainWindow final : public QMainWindow {
public:
    explicit MainWindow(QWidget* parent = nullptr);

    static const std::vector<FieldSpec>& all_fields();

private:
    using Params = std::variant<std::monostate, core::BevelSetParams,
                                core::SpurSetParams, core::HypoidSetParams,
                                core::PlanetarySetParams>;
    using Geometry = std::variant<std::monostate, core::bevel::SetGeometry,
                                  core::spur::SetGeometry,
                                  core::hypoid::SetGeometry,
                                  core::planetary::SetGeometry>;

    QComboBox* kind_combo_{};
    QComboBox* member_combo_{};
    QComboBox* scene_combo_{};
    QWidget* fields_panel_{};
    QTableWidget* derived_table_{};
    QTextEdit* validation_text_{};
    PreviewWidget* preview_widget_{};
    QLabel* scene_title_{};
    QLabel* status_label_{};
    QPushButton* build_button_{};
    QTimer* debounce_timer_{};

    std::map<std::string, QWidget*> field_controls_;
    std::map<std::string, QWidget*> field_rows_;
    Params params_;
    Geometry geometry_;
    core::ValidationResult validation_;
    preview::Scene2D scene_;
    bool has_geometry_{false};
    bool updating_widgets_{false};
    bool initialized_{false};
    std::string kind_key_{"bevel"};

    static const std::vector<FieldSpec>& fields_for(const std::string& key);
    static std::vector<std::pair<std::string, std::string>> scene_labels_for(
        const std::string& key);

    void create_widgets();
    void apply_kind(bool reset_parameters);
    void populate_from_params(const Params& params);
    void schedule_refresh();
    void refresh_now();
    void show_validation(const std::vector<std::string>& parse_errors);
    void show_derived();
    void show_scene();
    void set_status(const QString& text);

    [[nodiscard]] QString field_text(const char* key) const;
    void set_field_text(const char* key, const QString& text);
    [[nodiscard]] std::optional<double> read_optional_double(
        const FieldSpec& field, std::vector<std::string>& errors) const;
    [[nodiscard]] std::optional<double> read_double(
        const FieldSpec& field, std::vector<std::string>& errors) const;
    [[nodiscard]] std::optional<int> read_int(
        const FieldSpec& field, std::vector<std::string>& errors) const;
    [[nodiscard]] std::optional<std::string> read_choice(
        const FieldSpec& field, std::vector<std::string>& errors) const;
    [[nodiscard]] Params parse_params(std::vector<std::string>& errors) const;
    [[nodiscard]] Params default_params(const std::string& key,
                                        double module, int first, int second) const;
    [[nodiscard]] std::pair<int, int> counts(const Params& params) const;
    [[nodiscard]] double module(const Params& params) const;
    [[nodiscard]] std::string member_name() const;

    void on_kind_changed();
    void on_trace_changed();
    void auto_size();
    void fit_preview();
    void export_dxf();
    void export_csv();
    void load_preset();
    void save_preset();
    void build_in_solidworks();
};

} // namespace geargen::gui
