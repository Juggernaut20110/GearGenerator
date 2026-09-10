#include "gui/main_window.hpp"

#include <QApplication>

int main(int argc, char** argv)
{
    QApplication application(argc, argv);
    geargen::gui::MainWindow window;
    window.show();
    return application.exec();
}
