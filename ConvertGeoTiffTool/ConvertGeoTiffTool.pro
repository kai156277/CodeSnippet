#-------------------------------------------------
#
# Project created by QtCreator 2018-11-27T13:13:27
#
#-------------------------------------------------

# Qt Module
QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

# build target
TARGET = ConvertGeoTiffTool
TEMPLATE = app

# resource file && version info
RC_FILE += resource.rc

# define variable
DEFINES += Q_COMPILER_INITIALIZER_LISTS

# internationalization
TRANSLATIONS = ConvertGeoTiffTool_cn.ts

# ThirdParty lib
win32:CONFIG(release, debug|release): LIBS += -L"C:/ThirdPartyLib/libtiff-4.0.9/qdevlib" -ltiff
else:win32:CONFIG(debug, debug|release): LIBS += -L"C:/ThirdPartyLib/libtiff-4.0.9/qdevlib" -ltiffd

INCLUDEPATH += "C:/ThirdPartyLib/libtiff-4.0.9/include"
DEPENDPATH += "C:/ThirdPartyLib/libtiff-4.0.9/include"

win32:CONFIG(release, debug|release): LIBS += -L"C:/ThirdPartyLib/libgeotiff-1.4.2/qdevlib" -lgeotiff -lgeotiff_i
else:win32:CONFIG(debug, debug|release): LIBS += -L"C:/ThirdPartyLib/libgeotiff-1.4.2/qdevlib" -lgeotiffd -lgeotiff_id

INCLUDEPATH += "C:/ThirdPartyLib/libgeotiff-1.4.2/include"
DEPENDPATH += "C:/ThirdPartyLib/libgeotiff-1.4.2/include"

win32:CONFIG(release, debug|release): LIBS += -L"C:/ThirdPartyLib/libproj-5.0.1/qdevlib" -lproj_5_0
else:win32:CONFIG(debug, debug|release): LIBS += -L"C:/ThirdPartyLib/libproj-5.0.1/qdevlib" -lproj_5_0_d

INCLUDEPATH += "C:/ThirdPartyLib/libproj-5.0.1/include"
DEPENDPATH += "C:/ThirdPartyLib/libproj-5.0.1/include"

# src file
SOURCES += main.cpp\
    ConvertParamDialog.cpp \
    ConvertGeoTiffWidget.cpp

HEADERS  += \
    ConvertParamDialog.h \
    version.h \
    ConvertGeoTiffWidget.h

FORMS    += \
    ConvertParamDialog.ui \
    ConvertGeoTiffWidget.ui

RESOURCES += \
    image.qrc

