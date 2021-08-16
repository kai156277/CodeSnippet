#include "UiLogger.h"
#include "Widget.h"

#include <QApplication>

#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>
#include <thread>

Q_DECLARE_METATYPE(spdlog::level::level_enum)

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    Widget       w;
    w.show();
    qRegisterMetaType<spdlog::level::level_enum>();

    auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    console_sink->set_level(spdlog::level::trace);
    console_sink->set_pattern("[multi_sink_example] [%^%l%$] %v");

    auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>("logs/multisink.txt", true);
    file_sink->set_level(spdlog::level::trace);

    auto ui_sink = std::make_shared<ui_sink_mt>();
    ui_sink->set_level(spdlog::level::trace);
    //ui_sink->set_pattern("[%Y/%m/%d %H:%M:%S.%F] [%n] [%=8l] [thread %t] %v");

    QObject::connect(ui_sink.get()->msgRepeater(), &MsgRepeater::ui_log_msg, &w, &Widget::add_logging);
    QObject::connect(ui_sink.get()->msgRepeater(), &MsgRepeater::ui_log_flush, &w, &Widget::logging_flush);

    spdlog::sinks_init_list         sinks {console_sink, file_sink, ui_sink};
    std::shared_ptr<spdlog::logger> logger = std::make_shared<spdlog::logger>("multi_sink", sinks);
    spdlog::set_default_logger(logger);
    spdlog::set_level(spdlog::level::trace);

    spdlog::info("Welcome to spdlog!");
    spdlog::error("Some error message with arg: {}", 1);

    spdlog::trace("Easy padding in numbers like {:08d}", 12);
    spdlog::debug("Support for int: {0:d};  hex: {0:x};  oct: {0:o}; bin: {0:b}", 42);
    spdlog::info("Support for floats {:03.2f}", 1.23456);
    spdlog::warn("Positional args are {1} {0}..", "too", "supported");
    spdlog::error("{:<30}", "left aligned");
    spdlog::critical("Some debug message");

    return a.exec();
}

#if 0
    std::thread thread1([&logger]() {
        for (int i = 0; i < 10; ++i)
            spdlog::debug("thread1 test");
    });

    std::thread thread2([&logger]() {
        for (int i = 0; i < 10; ++i)
            spdlog::info("thread2 test");
    });

    std::thread thread3([&logger]() {
        for (int i = 0; i < 10; ++i)
            spdlog::warn("thread3 test");
    });

    std::thread thread4([&logger]() {
        for (int i = 0; i < 10; ++i)
            spdlog::error("thread4 test");
    });
    thread1.detach();
    thread2.detach();
    thread3.detach();
    thread4.detach();
#endif
