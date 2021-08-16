MODULE_NAME = ParseRieglStream

#include ($${PWD}/../../modules.pri)
#include ($${PWD}/../tools.pri)

LIB_PATH = C:/ThirdPartyLib/rivlib-2_5_9-x86_64-windows-vc141

win32:CONFIG(release, debug|release){
   LIBS += -L$${LIB_PATH}/lib/ -llibctrllib-mt
   LIBS += -L$${LIB_PATH}/lib/ -llibriboost_chrono-mt
   LIBS += -L$${LIB_PATH}/lib/ -llibriboost_date_time-mt
   LIBS += -L$${LIB_PATH}/lib/ -llibriboost_filesystem-mt
   LIBS += -L$${LIB_PATH}/lib/ -llibriboost_regex-mt
   LIBS += -L$${LIB_PATH}/lib/ -llibriboost_system-mt
   LIBS += -L$${LIB_PATH}/lib/ -llibriboost_thread-mt
   LIBS += -L$${LIB_PATH}/lib/ -llibscanlib-mt
   LIBS += -L$${LIB_PATH}/lib/ -lctrlifc-mt
   LIBS += -L$${LIB_PATH}/lib/ -lscanifc-mt
}
win32:CONFIG(debug, debug|release){
   LIBS += -L$${LIB_PATH}/lib/ -llibctrllib-mt-g
   LIBS += -L$${LIB_PATH}/lib/ -llibriboost_chrono-mt-g
   LIBS += -L$${LIB_PATH}/lib/ -llibriboost_date_time-mt-g
   LIBS += -L$${LIB_PATH}/lib/ -llibriboost_filesystem-mt-g
   LIBS += -L$${LIB_PATH}/lib/ -llibriboost_regex-mt-g
   LIBS += -L$${LIB_PATH}/lib/ -llibriboost_system-mt-g
   LIBS += -L$${LIB_PATH}/lib/ -llibriboost_thread-mt-g
   LIBS += -L$${LIB_PATH}/lib/ -llibscanlib-mt-g
   LIBS += -L$${LIB_PATH}/lib/ -lctrlifc-mt-g
   LIBS += -L$${LIB_PATH}/lib/ -lscanifc-mt-g
}

INCLUDEPATH += $${LIB_PATH}/include
DEPENDPATH += $${LIB_PATH}/include

INCLUDEPATH += $${PWD}
