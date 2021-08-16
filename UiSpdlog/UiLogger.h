#pragma once

#include <QDateTime>
#include <QObject>

#include <mutex>

#include <spdlog/details/null_mutex.h>
#include <spdlog/sinks/base_sink.h>

class MsgRepeater : public QObject
{
    Q_OBJECT
public:
    MsgRepeater(QObject *parent = nullptr)
        : QObject(parent)
    {
    }
    ~MsgRepeater() override = default;
signals:
    // void ui_log_msg(QString msg);
    void ui_log_msg(QString msg, spdlog::level::level_enum level);
    void ui_log_flush();
};

template <typename Mutex>
class ui_sink : public spdlog::sinks::base_sink<Mutex>
{
public:
    const MsgRepeater *msgRepeater() const
    {
        return const_cast<const MsgRepeater *>(&mMsgRepeater);
    }

protected:
    void sink_it_(const spdlog::details::log_msg &msg) override
    {
        spdlog::memory_buf_t formatted;
        base_sink<Mutex>::formatter_->format(msg, formatted);
        emit mMsgRepeater.ui_log_msg(QString::fromStdString(fmt::to_string(formatted)), msg.level);
    }

    void flush_() override
    {
        emit mMsgRepeater.ui_log_flush();
    }

private:
    MsgRepeater mMsgRepeater;
};

using ui_sink_mt = ui_sink<std::mutex>;
using ui_sin_st  = ui_sink<spdlog::details::null_mutex>;

#if 0
    QString loger_name(msg.logger_name.data());
    qDebug() << "logger_name:" << loger_name;
    qDebug() << "level:" << msg.level;
    qDebug() << "time_t:" << (std::chrono::system_clock::to_time_t(msg.time));
    qDebug() << "source| filename:" << msg.source.filename
             << "funcname:" << msg.source.funcname
             << "line:" << msg.source.line;
    qDebug() << "payload:" << QString(msg.payload.data());
    qDebug() << "current ms:" << QDateTime::currentMSecsSinceEpoch();
    qDebug() << "ms:" << spdlog::details::fmt_helper::time_fraction<std::chrono::milliseconds>(msg.time).count();
    qDebug() << "us:" << spdlog::details::fmt_helper::time_fraction<std::chrono::microseconds>(msg.time).count();
    qDebug() << "ns:" << spdlog::details::fmt_helper::time_fraction<std::chrono::nanoseconds>(msg.time).count();
    qDebug() << "-----------------------------";
#endif
