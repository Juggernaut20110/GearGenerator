#pragma once

#include "preview/scene2d/scene.hpp"

#include <QWidget>

class QPainter;

namespace geargen::gui {

class PreviewWidget final : public QWidget {
public:
    explicit PreviewWidget(QWidget* parent = nullptr);

    void set_scene(const preview::Scene2D& scene);
    void clear_scene();
    void fit_view();
    [[nodiscard]] const preview::Scene2D& scene() const noexcept { return scene_; }

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
    preview::View2D view_;
    QPoint last_drag_position_;
    bool dragging_{false};
    bool has_scene_{false};
    bool needs_fit_{true};

    void ensure_fit();
    void draw_legend(QPainter& painter) const;
    void draw_scale_bar(QPainter& painter) const;
};

} // namespace geargen::gui
