// process2cal.cpp: 定义应用程序的入口点。
//

#include "process2cal.h"

#include <fmt/format.h>
#include <QFileDialog>
#include <QString>
#include <QApplication>
#include <QStringList>
#include <QTextStream>
#include <QFile>
#include <QDebug>


constexpr double PI         = (3.14159265358979323846);   // 圆周率
constexpr double RAD_TO_DEG = (180.0 / PI);               // 弧度转度
constexpr double DEG_TO_RAD = (PI / 180.0);               // 度转弧度

using namespace std;

bool process2cal(const QString &file_name, const QString &suffix, double hA_offset, double vA_offset);
QStringList itemsOrderChange(const QStringList& process_items);

int main(int argc, char* argv[])
{
	QApplication a(argc, argv);
	QStringList process_file_names = QFileDialog::getOpenFileNames(nullptr, "open process cal output file");
	if(process_file_names.isEmpty())
	{
		qDebug() << "not open file" ;
		return 0;
	}

	const auto size =process_file_names.size();
	for(int i = 0; i < size; ++i)
	{
		fmt::print("{}/{}: ",i+1, size);
        process2cal(process_file_names[i], "cal", 90, 0);
	}

	cout << "Hello CMake." << endl;
	return 0;
}

bool process2cal(const QString &file_name, const QString &suffix, double hA_offset, double vA_offset)
{
	QFile process_cal_file(file_name);
	QFileInfo pcal_file_info(file_name);
	QString new_cal_file = pcal_file_info.absolutePath() + "/" + suffix + "_" + pcal_file_info.fileName();
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
	QStringList cal_items;
	while (!process_cal_file.atEnd())
	{
		line = process_cal_file.readLine().simplified();
		items = line.split(' ', Qt::SkipEmptyParts);
		if(items.size() != 25)
			continue;

		double hA = items[2].toDouble() * RAD_TO_DEG + hA_offset;
		double vA = items[3].toDouble() * RAD_TO_DEG + vA_offset;
        if(hA >360.0)
            hA -= 360.0;
		items[2] = QString::number(hA, 'f', 16);
		items[3] = QString::number(vA, 'f', 16);

		cal_items = itemsOrderChange(items);
		save_cal_stream << cal_items.join(',') << Qt::endl;
	}
	qDebug() << "finish write: " << new_cal_file;
	return true;
}

QStringList itemsOrderChange(const QStringList& process_items)
{
	QStringList cal_itmes;
	cal_itmes.reserve(process_items.size());

	// X,Y,Z_oe
	cal_itmes.push_back(process_items[22]);
	cal_itmes.push_back(process_items[23]);
	cal_itmes.push_back(process_items[24]);

	// time, r, hA, vA, std
	cal_itmes.push_back(process_items[0]);
	cal_itmes.push_back(process_items[1]);
	cal_itmes.push_back(process_items[2]);
	cal_itmes.push_back(process_items[3]);
	cal_itmes.push_back(process_items[4]);
	cal_itmes.push_back(process_items[5]);
	cal_itmes.push_back(process_items[6]);

	// r,p,y,std
	cal_itmes.push_back(process_items[7]);
	cal_itmes.push_back(process_items[8]);
	cal_itmes.push_back(process_items[9]);
	cal_itmes.push_back(process_items[10]);
	cal_itmes.push_back(process_items[11]);
	cal_itmes.push_back(process_items[12]);

	// blh
	cal_itmes.push_back(process_items[13]);
	cal_itmes.push_back(process_items[14]);
	cal_itmes.push_back(process_items[15]);

	// blh std
	cal_itmes.push_back(process_items[19]);
	cal_itmes.push_back(process_items[20]);
	cal_itmes.push_back(process_items[21]);

	cal_itmes.push_back(process_items[16]);
	cal_itmes.push_back(process_items[17]);
	cal_itmes.push_back(process_items[18]);
	return cal_itmes;
}
