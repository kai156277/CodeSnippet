#pragma once

#include <QSettings>
#include <QString>
#include <QWidget>

QT_BEGIN_NAMESPACE
namespace Ui {
class RegisterWidget;
}
QT_END_NAMESPACE

class RegisterWidget : public QWidget
{
    Q_OBJECT

public:
    RegisterWidget(QWidget *parent = nullptr);
    ~RegisterWidget();

private slots:
    void on_generatePushButton_clicked();

    void on_addSoftwarePushButton_clicked();

    void on_softwareInfoPushButton_clicked();

private:
    Ui::RegisterWidget *ui;
    QSettings           mSettings;
    QStringList         mSoftwareLists;
};
