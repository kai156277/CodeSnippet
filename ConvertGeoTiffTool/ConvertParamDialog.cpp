#include "ConvertParamDialog.h"
#include "ui_ConvertParamDialog.h"

#include <QPushButton>
#include <proj.h>
#include <geo_normalize.h>

static const int CGCS2000_Gauss_6N_13 		= 4491;
static const int CGCS2000_Gauss_6N_23 		= 4501;
static const int CGCS2000_Gauss_6N_CM_75E 	= 4502;
static const int CGCS2000_Gauss_6N_CM_135E 	= 4512;
static const int CGCS2000_Gauss_3N_25 		= 4513;
static const int CGCS2000_Gauss_3N_45 		= 4533;
static const int CGCS2000_Gauss_3N_CM_75E 	= 4534;
static const int CGCS2000_Gauss_3N_CM_135E 	= 4554;
static const int WGS84_UTM_1N 	= 32601;
static const int WGS84_UTM_60N 	= 32660;

QVector<int> longitudeToEPSG(double deg)
{
    QVector<int> nums;
    // WGS84 / UTM
    if(-180 == deg)
    {
        nums.push_back(WGS84_UTM_1N);
    }
    else if(-180 < deg && deg < 0)
    {
        nums.push_back((int)(deg / 6) + 32630);
    }
    else if(0 <= deg && deg < 180)
    {
        nums.push_back((int)(deg / 6) + 32631);
    }
    else if(deg == 180)
    {
        nums.push_back(WGS84_UTM_60N);
    }

    // CGCS2000 / Gauss-Kruger 6
    if(72 < deg && deg < 138)
    {
        int num = ((int)floor(deg) % 72) / 6;
        nums.push_back(num + CGCS2000_Gauss_6N_13);
        nums.push_back(num + CGCS2000_Gauss_6N_CM_75E);
    }

    // CGCS2000 / Gauss-Kruger 3
    if(73.5 < deg && deg < 136.5)
    {
        int num = ((int)round(deg) % 74) / 3;
        nums.push_back(num + CGCS2000_Gauss_3N_25);
        nums.push_back(num + CGCS2000_Gauss_3N_CM_75E);
    }
    return nums;
}

ConvertParamDialog::ConvertParamDialog(double lng, QWidget *parent) :
    QDialog(parent),
    ui(new Ui::ConvertParamDialog)
{
    ui->setupUi(this);
    ui->mLngSpinBox->setValue(lng);
    on_mLngSpinBox_valueChanged(lng);
}

ConvertParamDialog::~ConvertParamDialog()
{
    delete ui;
}

void ConvertParamDialog::on_mLngSpinBox_valueChanged(int arg1)
{
    QVector<int> epsgs = longitudeToEPSG(arg1);
    ui->mExportCoordSysComboBox->clear();
    char* EPSGName;
    for(int epsg: epsgs)
    {
        if(GTIFGetPCSInfo(epsg, &EPSGName, nullptr, nullptr, nullptr) == 1)
            ui->mExportCoordSysComboBox->addItem(QString("%1 (EPSG: %2)").arg(QString(EPSGName)).arg(epsg), epsg);
    }

    ui->mExportCoordSysComboBox->setCurrentIndex(0);
}

void ConvertParamDialog::on_mExportCoordSysComboBox_currentIndexChanged(int index)
{
    bool ok = false;
    int epsg = ui->mExportCoordSysComboBox->itemData(index).toInt(&ok);
    if(!ok)
    {
        ui->buttonBox->button(QDialogButtonBox::Ok)->setEnabled(false);
        return ;
    }
    else
        ui->buttonBox->button(QDialogButtonBox::Ok)->setEnabled(true);


    QString EPSGPath = "+init=" + QApplication::applicationDirPath() + "/share/" + QString("epsg:%1").arg(epsg);

    PJ* proj = proj_create(0, EPSGPath.toLocal8Bit().data());
    if(proj)
    {
        PJ_PROJ_INFO info = proj_pj_info(proj);
        QStringList params = QString(info.definition).split(" ", QString::SkipEmptyParts);
        QString projparam;
        if(!params.isEmpty())
        {
            params[0] = "";
            projparam = params.join(" +");
        }
        ui->mProj4StrLabel->setText(projparam);
    }
}

void ConvertParamDialog::on_buttonBox_accepted()
{
    int index = ui->mExportCoordSysComboBox->currentIndex();
    epsg = ui->mExportCoordSysComboBox->itemData(index).toInt();
}
