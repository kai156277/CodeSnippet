#include <QApplication>
#include <QDebug>
#include <QFileDialog>
#include <QFileInfo>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    QStringList files = QFileDialog::getOpenFileNames(nullptr, "txt remove car", "D:/");
    if (!files.isEmpty())
    {
        for (int i = 0; i < files.size(); ++i)
        {
            QFileInfo file_info(files[i]);
            QString   old_file   = file_info.absoluteFilePath();
            int       last_index = file_info.baseName().lastIndexOf("_car");
            QString   new_file   = file_info.absolutePath() + "/" + file_info.baseName().mid(0, last_index) + ".txt";
            if (QFile::rename(old_file, new_file))
                qDebug() << old_file << "->" << new_file;
        }
    }

    qDebug() << "END!";
    return 0;
}
