#pragma once

#include <QApplication>
#include <QThread>
#include <QTimer>
#include <iostream>
#include <thread>

class TestThread : public QThread
{
    Q_OBJECT
public:
    TestThread(QObject *parent = nullptr)
        : QThread(parent)
    {
        QTimer::singleShot(1000, [this]() {
            exit(0);
        });
    }
    void run() override
    {

        std::cout << "hello word in " << std::this_thread::get_id() << std::endl;
        exec();
        std::cout << "exe word in " << std::this_thread::get_id() << std::endl;
    }
};
