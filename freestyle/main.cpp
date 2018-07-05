#include "app/Controller.h"
#include "app/AppMainWindow.h"
#include "app/AppConfig.h"
#include <QGLFormat>

int main(int argc, char** argv)
{
    // sets the paths
    QApplication::setColorSpec(QApplication::ManyColor);
    QApplication *app = new QApplication(argc, argv);
    Q_INIT_RESOURCE(freestyle);

    Config::Path pathconfig;

    QGLFormat myformat;
    myformat.setAlpha(true);
    QGLFormat::setDefaultFormat( myformat );

    AppMainWindow mainWindow(NULL, "Freestyle");

    g_pController = new Controller;
    g_pController->SetMainWindow(&mainWindow);
    g_pController->SetView(mainWindow.pQGLWidget);

    mainWindow.show();

    int res = app->exec();

    delete g_pController; 

    return res;
}
