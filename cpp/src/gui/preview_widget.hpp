#pragma once

#include "preview/scene2d/scene.hpp"
#include "preview/scene3d/scene.hpp"

#include <QWidget>

class QPainter;

namespace geargen::gui {

class PreviewWidget final : public QWidget {
public:
    explicit PreviewWidget(QWidget* parent = nullptr);

    void set_scene(const preview::Scene2D& scene);
    void set_scene3d(const preview::Scene3D& scene, bool refit = true);
    void clear_scene();
    void fit_view();
    void set_camera_view(const std::string& name);
    [[nodiscard]] bool is_3d() const noexcept { return is_3d_; }
    [[nodiscard]] const preview::Scene2D& scene() const noexcept { return scene_; }
    [[nodiscard]] const preview::Scene3D& scene3d() const noexcept { return scene3d_; }

protected:
    void paintEvent(QPaintEvent* event) override;
    void resizeEvent(QResizeEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;
    void mouseDoubleClickEvent(QMouseEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;

private:
    preview::Scene2D scene_;
    preview::Scene3D scene3d_;
    preview::View2D view_;
    preview::Camera camera_;
    QPoint last_drag_position_;
    Qt::MouseButton drag_button_{Qt::NoButton};
    bool dragging_{false};
    bool has_scene_{false};
    bool is_3d_{false};
    bool needs_fit_{true};

    void ensure_fit();
    void draw_legend(QPainter& painter) const;
    void draw_legend3d(QPainter& painter) const;
    void draw_scale_bar(QPainter& painter) const;
};

} // namespace geargen::gui
