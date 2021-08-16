#pragma once

#include <QtWidgets/QMainWindow>
#include "ui_DefineProfileTest.h"

class DefineProfileTest : public QMainWindow
{
	Q_OBJECT

public:
	DefineProfileTest(QWidget *parent = Q_NULLPTR);

private:
	Ui::DefineProfileTestClass ui;
};
