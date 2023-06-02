// obs2ctrl.cpp: 目标的源文件。
//

#include "obs2ctrl.h"

#include <QApplication>
#include <qstring.h>
#include <qstringlist.h>
#include <qfiledialog.h>
#include <qdebug.h>

#include <fmt/format.h>

bool obs2ctrl(const QString& file_name);

int main(int argc, char* argv[])
{
	QApplication a(argc, argv);
	QStringList process_file_names = QFileDialog::getOpenFileNames(nullptr, "open cal files", "E:/Calibration");
	if(process_file_names.isEmpty())
	{
		qDebug() << "not open file" ;
		return 0;
	}

	const auto size =process_file_names.size();
	for(int i = 0; i < size; ++i)
	{
		fmt::print("{}/{}: ",i+1, size);
		obs2ctrl(process_file_names[i]);
	}

	fmt::print("End!");
	return 0;
}

bool obs2ctrl(const QString& file_name)
{
	QFile process_cal_file(file_name);
	QFileInfo pcal_file_info(file_name);
	QString new_cal_file = pcal_file_info.absolutePath() + "/ctrl/" + pcal_file_info.fileName();
	QFile save_cal_file(new_cal_file);

	if(!process_cal_file.open(QIODevice::ReadOnly))
	{
		qDebug() << "can`t open VSursProcess cal file: " << file_name ;
		return false;
	}
	if(!save_cal_file.open(QIODevice::WriteOnly))
	{
		qDebug() << "can`t open new cal file: " << new_cal_file;
		return false;
	}
	QTextStream save_cal_stream(&save_cal_file);

	QString line;
	QStringList items;
	
	while (!process_cal_file.atEnd())
	{
		line = process_cal_file.readLine().simplified();
		items = line.split(',', Qt::SkipEmptyParts);
		if(items.size() < 4)
			continue;

//		QStringList cal_items ={items[3] , items[0] , items[1] , items[2]};
         double hA = items[5].toDouble();
        hA += 90.0;
        if(hA >360.0)
            hA -= 360.0;

        items[5] = QString::number(hA, 'f', 14);

		save_cal_stream << items.join(',') << Qt::endl;
	}
	qDebug() << "finish write: " << new_cal_file;
	return true;
}
