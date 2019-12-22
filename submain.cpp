#include "mulplayer.h"
#include <QApplication>

int submain(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MulPlayer w;
    w.show();

    return a.exec();
}
