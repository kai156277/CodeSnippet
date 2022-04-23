QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++11 console

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
    CheckPage.cpp \
    MainWindow.cpp \
    MultiBeamFilePage.cpp \
    MultiBeamProgressWidget.cpp \
    MultiBeamWizard.cpp \
    ProgressWidget.cpp \
    ProjectMgr.cpp \
    ScanCheckPage.cpp \
    ScanFilePage.cpp \
    ScanFilterPage.cpp \
    ScanOutPage.cpp \
    ScanProgressWidget.cpp \
    ScanWizard.cpp \
    WizardPage.cpp \
    WizardWidget.cpp \
    main.cpp

HEADERS += \
    CheckPage.h \
    MainWindow.h \
    MultiBeamFilePage.h \
    MultiBeamProgressWidget.h \
    MultiBeamWizard.h \
    ProgressWidget.h \
    ProjectMgr.h \
    ScanCheckPage.h \
    ScanFilePage.h \
    ScanFilterPage.h \
    ScanOutPage.h \
    ScanProgressWidget.h \
    ScanWizard.h \
    WizardPage.h \
    WizardWidget.h

FORMS += \
    CheckPage.ui \
    MainWindow.ui \
    ProgressWidget.ui \
    ScanFilePage.ui \
    ScanFilterPage.ui \
    ScanOutPage.ui \
    WizardPage.ui \
    WizardWidget.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

#win32:CONFIG(debug, debug|release):{
#QMAKE_CFLAGS_DEBUG += /MTd
#QMAKE_CXXFLAGS_DEBUG += /MTd
#}
#else:win32:CONFIG(release, debug|release):{
#QMAKE_CFLAGS_RELEASE += /MT
#QMAKE_CXXFLAGS_RELEASE += /MT /NODEFAULTLIB:msvcrt.lib
#}


INCLUDEPATH += C:/ThirdPartyLib/libVSurs0415/include
DEPENDPATH += C:/ThirdPartyLib/libVSurs0415/include


LIBS += -LC:/ThirdPartyLib/libVSurs0415/lib/release/ -lBaseProcThread
INCLUDEPATH += C:/ThirdPartyLib/libVSurs0415/include/BaseProcThread
DEPENDPATH += C:/ThirdPartyLib/libVSurs0415/include/BaseProcThread

LIBS += -LC:/ThirdPartyLib/libVSurs0415/lib/release/ -lCoordinateTransform
INCLUDEPATH += C:/ThirdPartyLib/libVSurs0415/include/CoordinateTransform
DEPENDPATH += C:/ThirdPartyLib/libVSurs0415/include/CoordinateTransform

LIBS += -LC:/ThirdPartyLib/libVSurs0415/lib/release/ -lcloudui
INCLUDEPATH += C:/ThirdPartyLib/libVSurs0415/include/cloudui
DEPENDPATH += C:/ThirdPartyLib/libVSurs0415/include/cloudui

LIBS += -LC:/ThirdPartyLib/libVSurs0415/lib/release/ -lxs3d-core
INCLUDEPATH += C:/ThirdPartyLib/libVSurs0415/include/xs3d/include
DEPENDPATH += C:/ThirdPartyLib/libVSurs0415/include/xs3d/include

INCLUDEPATH += C:/ThirdPartyLib/libVSurs0415/include/common
DEPENDPATH += C:/ThirdPartyLib/libVSurs0415/include/common
LIBS += -LC:/ThirdPartyLib/libVSurs0415/lib/release/ -lxs-common
PRE_TARGETDEPS += C:/ThirdPartyLib/libVSurs0415/lib/release/xs-common.lib

INCLUDEPATH += C:/ThirdPartyLib/libVSurs0415/include/comui
DEPENDPATH += C:/ThirdPartyLib/libVSurs0415/include/comui
LIBS += -LC:/ThirdPartyLib/libVSurs0415/lib/release/ -lxs-comui
PRE_TARGETDEPS += C:/ThirdPartyLib/libVSurs0415/lib/release/xs-comui.lib
