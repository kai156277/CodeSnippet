QT += core gui widgets
TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle

SOURCES += main.cpp

win32:CONFIG(release, debug|release): LIBS += -LC:/ThirdPartyLib/vsurs0415/vsurs/lib/release/ -lBaseProcThread
else:win32:CONFIG(debug, debug|release): LIBS += -LC:/ThirdPartyLib/vsurs0415/vsurs/lib/debug/ -lBaseProcThread

INCLUDEPATH += C:/ThirdPartyLib/vsurs0415/vsurs/include
DEPENDPATH += C:/ThirdPartyLib/vsurs0415/vsurs/include

INCLUDEPATH += C:/ThirdPartyLib/vsurs0415/vsurs/include/common
DEPENDPATH += C:/ThirdPartyLib/vsurs0415/vsurs/include/common
