QT += gui widgets

CONFIG += c++11 console
CONFIG -= app_bundle

# The following define makes your compiler emit warnings if you use
# any Qt feature that has been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
        EllipseFit.cpp \
        EllipseRANSAC.cpp \
        LASFile.cpp \
        PCDFile.cpp \
        TunnelCrossSection.cpp \
        TunnelPointCloud.cpp \
        TunnelRing.cpp \
        main.cpp

HEADERS += \
    EllipseFit.h \
    EllipseRANSAC.h \
    LASFile.h \
    PCDFile.h \
    PointXYZIT.h \
    TunnelCrossSection.h \
    TunnelPointCloud.h \
    TunnelRing.h \
    stdafx.h

include(pcl.pri)

msvc {
    QMAKE_CFLAGS += /utf-8
    QMAKE_CXXFLAGS += /utf-8
}

win32:CONFIG(release, debug|release):{
    LIBS += -LC:/ThirdPartyLib/libLAStools/lib/LASlib/release/ -lLASlib
    PRE_TARGETDEPS += C:/ThirdPartyLib/libLAStools/lib/LASlib/release/LASlib.lib
}
else:win32:CONFIG(debug, debug|release):{
    LIBS += -LC:/ThirdPartyLib/libLAStools/lib/LASlib/debug/ -lLASlib
    PRE_TARGETDEPS += C:/ThirdPartyLib/libLAStools/lib/LASlib/debug/LASlib.lib
}

INCLUDEPATH += C:/ThirdPartyLib/libLAStools/include
DEPENDPATH += C:/ThirdPartyLib/libLAStools/include

INCLUDEPATH += C:/ThirdPartyLib/spdlog
DEPENDPATH += C:/ThirdPartyLib/spdlog



DEFINES += NOMINMAX

# 使用预编译头加快编译速度
CONFIG += precompiled_header
PRECOMPILED_HEADER = stdafx.h

# 多线程编译
QMAKE_CXXFLAGS += /MP
