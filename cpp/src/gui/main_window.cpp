#include "gui/main_window.hpp"

#include <QLabel>
#include <QVBoxLayout>
#include <QWidget>

namespace geargen::gui {

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent)
{
    setWindowTitle(QStringLiteral("Gear Generator - native port"));
    resize(720, 420);

    auto* central = new QWidget(this);
    auto* layout = new QVBoxLayout(central);
    auto* label = new QLabel(
        QStringLiteral("Native C++/Qt toolchain is working.\n"
                       "Gear mathematics will be ported against the Python reference."),
        central);
    label->setAlignment(Qt::AlignCenter);
    layout->addWidget(label);
    setCentralWidget(central);
}

} // namespace geargen::gui
