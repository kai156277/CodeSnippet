#include "palette.h"
#include <QBoxLayout>
#include<QDebug>
#pragma execution_character_set("utf-8")

Palette::Palette(QWidget *parent)
    : QDialog(parent)
{

    this->setWindowTitle("显示设置");
    createContentFrame();
    QHBoxLayout *mainLayout = new QHBoxLayout(this);
    //mainLayout->addWidget(ctrlFrame);
    mainLayout->addWidget(contentFrame);
    connect(okBtn,SIGNAL(clicked(bool)),this,SLOT(Save()));
    connect(cancelBtn,SIGNAL(clicked(bool)),this,SLOT(Cancel()));
}
void Palette::Save()
{

    QStringList colorList = QColor::colorNames();
    BackColor = QColor(colorList[windowComboBox->currentIndex()]);
    PointColor= QColor(colorList[windowTextComboBox->currentIndex()]);
    PointSize=comboBox1->currentIndex()+1;
    qDebug()<<PointSize;
    qDebug()<<"BackColor";
    qDebug()<<(int)BackColor.red();
    qDebug()<<(int)BackColor.green();
    qDebug()<<(int)BackColor.blue();
    qDebug()<<"PointColor";
    qDebug()<<(int)PointColor.red();
    qDebug()<<(int)PointColor.green();
    qDebug()<<(int)PointColor.blue();
    update=1;
    this->close();
}
void Palette::Cancel()
{
    this->close();

}

void Palette::createCtrlFrame()
{
    /*ctrlFrame = new QFrame; //窗口背景色
    windowLabel = new QLabel(tr("背景颜色: "));

    windowComboBox = new QComboBox;
    fillColorList(windowComboBox);
    connect(windowComboBox,SIGNAL(activated(int)),this,SLOT(showWindow()));

    windowTextLabel = new QLabel(tr("点云颜色: ")); //窗口前景色
    windowTextComboBox = new QComboBox;
    fillColorList(windowTextComboBox);
    connect(windowTextComboBox,SIGNAL(activated(int)),this,SLOT(showWindowText()));


    QGridLayout *mainLayout = new QGridLayout(ctrlFrame);
    mainLayout->setSpacing(20);
    mainLayout->addWidget(windowLabel,1,0);
    mainLayout->addWidget(windowComboBox,1,1);

    mainLayout->addWidget(windowTextLabel,2,0);
    mainLayout->addWidget(windowTextComboBox,2,1);*/

}

void Palette::createContentFrame()
{
    contentFrame = new QFrame;
    label1 = new QLabel(tr("点云大小："));
    comboBox1 = new QComboBox;
    for(int i=1;i<9;i++)
    {
        comboBox1->addItem(QString::number(i));
    }

    //ctrlFrame = new QFrame; //窗口背景色
    windowLabel = new QLabel(tr("背景颜色: "));
    windowComboBox = new QComboBox;
    fillColorList(windowComboBox);
    //connect(windowComboBox,SIGNAL(activated(int)),this,SLOT(showWindow()));

    windowTextLabel = new QLabel(tr("点云颜色: ")); //窗口前景色
    windowTextComboBox = new QComboBox;
    fillColorList(windowTextComboBox);
    //connect(windowTextComboBox,SIGNAL(activated(int)),this,SLOT(showWindowText()));

    QGridLayout *topLayout = new QGridLayout;
    topLayout->addWidget(label1,0,0);
    topLayout->addWidget(comboBox1,0,1);

    topLayout->addWidget(windowLabel,1,0);
    topLayout->addWidget(windowComboBox,1,1);

    topLayout->addWidget(windowTextLabel,2,0);
    topLayout->addWidget(windowTextComboBox,2,1);


    okBtn = new QPushButton(tr("确认"));
    cancelBtn = new QPushButton(tr("取消"));

    QHBoxLayout *bottomLayout = new QHBoxLayout;
    bottomLayout->addStretch(1);
    bottomLayout->addWidget(okBtn);
    bottomLayout->addWidget(cancelBtn);

    QVBoxLayout *mainlayout = new QVBoxLayout(contentFrame);
    mainlayout->addLayout(topLayout);
    mainlayout->addLayout(bottomLayout);

    okBtn->setAutoFillBackground(true);     //允许自动填充
    cancelBtn->setAutoFillBackground(true);
    contentFrame->setAutoFillBackground(true);
}


void Palette::fillColorList(QComboBox *comboBox)
{
    QStringList colorList = QColor::colorNames();
    QString color;

    foreach (color, colorList)
    {
        QPixmap pix(QSize(70,20));
        pix.fill(QColor(color));
        comboBox->addItem(QIcon(pix), NULL);
        comboBox->setIconSize(QSize(70,20));
        comboBox->setSizeAdjustPolicy(QComboBox::AdjustToContents);
    }
}

Palette::~Palette()
{

}
