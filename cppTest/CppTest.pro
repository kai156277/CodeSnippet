QT += core gui widgets
TEMPLATE = app
CONFIG += console c++17
CONFIG -= app_bundle

SOURCES += main.cpp \
    Dialog.cpp

INCLUDEPATH += C:/ThirdPartyLib/libspdlog-1.8.2/include
DEPENDPATH += C:/ThirdPartyLib/libspdlog-1.8.2/include

msvc {
    QMAKE_CFLAGS += /utf-8
    QMAKE_CXXFLAGS += /utf-8
}

FORMS += \
    Dialog.ui

HEADERS += \
    Dialog.h
