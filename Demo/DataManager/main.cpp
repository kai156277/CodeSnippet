#include <QtCore/qdebug.h>
#include <QtGui/qguiapplication.h>

#include "DataManager.h"
#include "DataSource.h"
#include "DefineExportProfileDialog.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    pos::DataManager::Instance()->RegisterDataSource(new pos::RandomTimeData());
    DefineExportProfileDialog defineExport;

    defineExport.SetFileExtensionName("txt");
    defineExport.show();
    defineExport.AddDataSource("RandomTime");

    return a.exec();
}
