#include <QApplication>

#include <QDebug>
#include <QFile>
#include <QFileDialog>
#include <QTextStream>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    QStringList feature_names = {"J1D", "J1DN", "J1L", "J1M", "J1R", "LQ-sphere", "RQ-sphere", "SZSF", "XGWS", "WJ1W", "EJ1E"};

    QStringList files = QFileDialog::getOpenFileNames(nullptr, "open J1 gnss feature", "E:\\Calibration\\ST");

    QFile merge_txt = "feature_merge.txt";

    if (!merge_txt.open(QIODevice::WriteOnly))
    {
        qDebug() << "can`t open merge txt";
        return 1;
    }
    QTextStream write(&merge_txt);

    for (int i = 0; i < files.size(); ++i)
    {
        QFile     feature_file(files[i]);
        QFileInfo file_name(files[i]);

        if (!feature_file.open(QIODevice::ReadOnly))
        {
            qDebug() << "can`t open feature file: " << files[i];
            continue;
        }

        // QString feature_name = file_name.baseName().split("_").last();
        QString feature_name = file_name.baseName();

        int num = feature_names.indexOf(feature_name);

        while (!feature_file.atEnd())
        {
            QString     line  = feature_file.readLine().simplified();
            QStringList items = line.split(',', QString::SkipEmptyParts);
            items.append(QString::number(num));
            write << items.join(',') << endl;

            // line.append(" " + QString::number(num));
            // write << line << endl;
        }
    }

    qDebug() << "END!";
    return 0;
}
