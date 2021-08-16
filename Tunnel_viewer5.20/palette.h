#ifndef PALETTE_H
#define PALETTE_H

#include <QDialog>
#include <QComboBox>
#include <QLabel>
#include <QTextEdit>
#include <QPushButton>
#include <QLineEdit>
#include <QColor>
class Palette : public QDialog
{
    Q_OBJECT

public:
    Palette(QWidget *parent = 0);
    ~Palette();
    QColor PointColor=QColor(0,0,0);
    QColor BackColor=QColor(255,255,255);
    int PointSize=-1;
    void createCtrlFrame();
    void createContentFrame();
    void fillColorList(QComboBox *);
    int update=0;

private slots:
    void Save();
    void Cancel();


private:
    QLabel *windowLabel;
    QComboBox *windowComboBox;
    QLabel *windowTextLabel;
    QComboBox *windowTextComboBox;


    QFrame *contentFrame;
    QLabel *label1;
    QComboBox *comboBox1;


    QPushButton *okBtn;
    QPushButton *cancelBtn;
};

#endif // PALETTE_H
