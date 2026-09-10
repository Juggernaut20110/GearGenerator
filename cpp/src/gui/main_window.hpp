#pragma once

#include <QMainWindow>

namespace geargen::gui {

class MainWindow final : public QMainWindow {
public:
    explicit MainWindow(QWidget* parent = nullptr);
};

} // namespace geargen::gui
