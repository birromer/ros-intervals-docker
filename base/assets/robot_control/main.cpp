#include <QApplication>
#include "robotcontrol.h"

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    RobotControl rbctrl;
    rbctrl.show();

    app.exec();

    return 0;
}
