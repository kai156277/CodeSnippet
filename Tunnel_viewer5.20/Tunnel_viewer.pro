#-------------------------------------------------
#
# Project created by QtCreator 2020-02-26T14:44:29
#
#-------------------------------------------------
windows {
     DEFINES *= Q_COMPILER_INITIALIZER_LISTS
}
QT += core xml opengl gui widgets

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = Tunnel_viewer
TEMPLATE = app
CONFIG += qaxcontainer
#msvc {
#    QMAKE_CFLAGS += /utf-8
#    QMAKE_CXXFLAGS += /utf-8
#}

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0


SOURCES += \
    SectionFitting.cpp \
    SectionFittingDialog.cpp \
    TunnelTypeDialog.cpp \
        main.cpp \
        mainwindow.cpp \
    dialog_section.cpp \
    dialog_sections.cpp \
    dialog_about.cpp \
    dialog_attribute.cpp \
    ExtractionDialog.cpp \
    AxisDialog.cpp \
    palette.cpp

HEADERS += \
    SectionFitting.h \
    SectionFittingDialog.h \
    TunnelDefType.h \
    TunnelTypeDialog.h \
        mainwindow.h \
    dialog_section.h \
    dialog_sections.h \
    dialog_about.h \
    dialog_attribute.h \
    ExtractionDialog.h \
    AxisDialog.h \
    excelhelper.h \
    palette.h

FORMS += \
    SectionFittingDialog.ui \
    TunnelTypeDialog.ui \
        mainwindow.ui \
    dialog_section.ui \
    dialog_sections.ui \
    dialog_about.ui \
    dialog_attribute.ui \
    ExtractionDialog.ui \
    AxisDialog.ui \
    palette.ui

#PCL_ROOT_PATH = 'C:/Program Files/PCL 1.8.1'

#INCLUDEPATH += '$${PCL_ROOT_PATH}/include/pcl-1.8'



INCLUDEPATH += F:/PCL/PCL1.8.0/include/pcl-1.8\
INCLUDEPATH += F:/PCL/PCL1.8.0/include/pcl-1.8/pcl\
INCLUDEPATH += F:/PCL/PCL1.8.0/3rdParty/Boost/include/boost-1_59\
INCLUDEPATH += F:/PCL/PCL1.8.0/3rdParty/Eigen/eigen3\
INCLUDEPATH += F:/PCL/PCL1.8.0/3rdParty/FLANN/include\
INCLUDEPATH += F:/PCL/PCL1.8.0/3rdParty/FLANN/include/flann\
INCLUDEPATH += F:/PCL/PCL1.8.0/3rdParty/OpenNI2/Include\
INCLUDEPATH += F:/PCL/PCL1.8.0/3rdParty/Qhull/include\
INCLUDEPATH += F:/PCL/PCL1.8.0/3rdParty/VTK/include/vtk-7.0\
INCLUDEPATH += F:/libQGLViewer/MSVC13_32bit_qt5.9.1



CONFIG(debug,debug|release){
LIBS += -LF:/libQGLViewer/MSVC13_32bit_qt5.9.1/QGLViewer\
        -lQGLViewerd2
LIBS += -lopengl32 \
    -lglu32
LIBS += -LF:/PCL/PCL1.8.0/lib\
        -lpcl_apps_debug\
        -lpcl_common_debug\
        -lpcl_features_debug\
        -lpcl_filters_debug\
        -lpcl_io_debug\
        -lpcl_io_ply_debug\
        -lpcl_kdtree_debug\
        -lpcl_keypoints_debug\
        -lpcl_ml_debug\
        -lpcl_octree_debug\
        -lpcl_outofcore_debug\
        -lpcl_people_debug\
        -lpcl_recognition_debug\
        -lpcl_registration_debug\
        -lpcl_sample_consensus_debug\
        -lpcl_search_debug\
        -lpcl_segmentation_debug\
        -lpcl_simulation_debug\
        -lpcl_stereo_debug\
        -lpcl_surface_debug\
        -lpcl_tracking_debug\
        -lpcl_visualization_debug\

LIBS += -LF:/PCL/PCL1.8.0/3rdParty/Boost/lib\
        -llibboost_atomic-vc120-mt-gd-1_59\
        -llibboost_chrono-vc120-mt-gd-1_59\
        -llibboost_container-vc120-mt-gd-1_59\
        -llibboost_context-vc120-mt-gd-1_59\
        -llibboost_coroutine-vc120-mt-gd-1_59\
        -llibboost_date_time-vc120-mt-gd-1_59\
        -llibboost_exception-vc120-mt-gd-1_59\
        -llibboost_filesystem-vc120-mt-gd-1_59\
        -llibboost_graph-vc120-mt-gd-1_59\
        -llibboost_iostreams-vc120-mt-gd-1_59\
        -llibboost_locale-vc120-mt-gd-1_59\
        -llibboost_log-vc120-mt-gd-1_59\
        -llibboost_log_setup-vc120-mt-gd-1_59\
        -llibboost_math_c99-vc120-mt-gd-1_59\
        -llibboost_math_c99f-vc120-mt-gd-1_59\
        -llibboost_math_c99l-vc120-mt-gd-1_59\
        -llibboost_math_tr1-vc120-mt-gd-1_59\
        -llibboost_math_tr1f-vc120-mt-gd-1_59\
        -llibboost_math_tr1l-vc120-mt-gd-1_59\
        -llibboost_mpi-vc120-mt-gd-1_59\
        -llibboost_prg_exec_monitor-vc120-mt-gd-1_59\
        -llibboost_program_options-vc120-mt-gd-1_59\
        -llibboost_random-vc120-mt-gd-1_59\
        -llibboost_regex-vc120-mt-gd-1_59\
        -llibboost_serialization-vc120-mt-gd-1_59\
        -llibboost_signals-vc120-mt-gd-1_59\
        -llibboost_system-vc120-mt-gd-1_59\
        -llibboost_test_exec_monitor-vc120-mt-gd-1_59\
        -llibboost_thread-vc120-mt-gd-1_59\
        -llibboost_timer-vc120-mt-gd-1_59\
        -llibboost_unit_test_framework-vc120-mt-gd-1_59\
        -llibboost_wave-vc120-mt-gd-1_59\
        -llibboost_wserialization-vc120-mt-gd-1_59

LIBS += -LF:/PCL/PCL1.8.0/3rdParty/FLANN/lib\
        -lflann-gd\
        -lflann_cpp_s-gd\
        -lflann_s-gd

LIBS += -LF:/PCL/PCL1.8.0/3rdParty/OpenNI2/Lib\
        -lOpenNI2

LIBS += -LF:/PCL/PCL1.8.0/3rdParty/Qhull/lib\
        -lqhull-gd\
        -lqhullcpp-gd\
        -lqhullstatic-gd\
        -lqhullstatic_r-gd\
        -lqhull_p-gd\
        -lqhull_r-gd

LIBS += -LF:/PCL/PCL1.8.0/3rdParty/VTK/lib\
        -lvtkalglib-7.0-gd\
        -lvtkChartsCore-7.0-gd\
        -lvtkCommonColor-7.0-gd\
        -lvtkCommonComputationalGeometry-7.0-gd\
        -lvtkCommonCore-7.0-gd\
        -lvtkCommonDataModel-7.0-gd\
        -lvtkCommonExecutionModel-7.0-gd\
        -lvtkCommonMath-7.0-gd\
        -lvtkCommonMisc-7.0-gd\
        -lvtkCommonSystem-7.0-gd\
        -lvtkCommonTransforms-7.0-gd\
        -lvtkDICOMParser-7.0-gd\
        -lvtkDomainsChemistry-7.0-gd\
        -lvtkDomainsChemistryOpenGL2-7.0-gd\
        -lvtkexoIIc-7.0-gd\
        -lvtkexpat-7.0-gd\
        -lvtkFiltersAMR-7.0-gd\
        -lvtkFiltersCore-7.0-gd\
        -lvtkFiltersExtraction-7.0-gd\
        -lvtkFiltersFlowPaths-7.0-gd\
        -lvtkFiltersGeneral-7.0-gd\
        -lvtkFiltersGeneric-7.0-gd\
        -lvtkFiltersGeometry-7.0-gd\
        -lvtkFiltersHybrid-7.0-gd\
        -lvtkFiltersHyperTree-7.0-gd\
        -lvtkFiltersImaging-7.0-gd\
        -lvtkFiltersModeling-7.0-gd\
        -lvtkFiltersParallel-7.0-gd\
        -lvtkFiltersParallelImaging-7.0-gd\
        -lvtkFiltersProgrammable-7.0-gd\
        -lvtkFiltersSelection-7.0-gd\
        -lvtkFiltersSMP-7.0-gd\
        -lvtkFiltersSources-7.0-gd\
        -lvtkFiltersStatistics-7.0-gd\
        -lvtkFiltersTexture-7.0-gd\
        -lvtkFiltersVerdict-7.0-gd\
        -lvtkfreetype-7.0-gd\
        -lvtkGeovisCore-7.0-gd\
        -lvtkglew-7.0-gd\
        -lvtkGUISupportQt-7.0-gd\
        -lvtkGUISupportQtSQL-7.0-gd\
        -lvtkhdf5-7.0-gd\
        -lvtkhdf5_hl-7.0-gd\
        -lvtkImagingColor-7.0-gd\
        -lvtkImagingCore-7.0-gd\
        -lvtkImagingFourier-7.0-gd\
        -lvtkImagingGeneral-7.0-gd\
        -lvtkImagingHybrid-7.0-gd\
        -lvtkImagingMath-7.0-gd\
        -lvtkImagingMorphological-7.0-gd\
        -lvtkImagingSources-7.0-gd\
        -lvtkImagingStatistics-7.0-gd\
        -lvtkImagingStencil-7.0-gd\
        -lvtkInfovisCore-7.0-gd\
        -lvtkInfovisLayout-7.0-gd\
        -lvtkInteractionImage-7.0-gd\
        -lvtkInteractionStyle-7.0-gd\
        -lvtkInteractionWidgets-7.0-gd\
        -lvtkIOAMR-7.0-gd\
        -lvtkIOCore-7.0-gd\
        -lvtkIOEnSight-7.0-gd\
        -lvtkIOExodus-7.0-gd\
        -lvtkIOExport-7.0-gd\
        -lvtkIOGeometry-7.0-gd\
        -lvtkIOImage-7.0-gd\
        -lvtkIOImport-7.0-gd\
        -lvtkIOInfovis-7.0-gd\
        -lvtkIOLegacy-7.0-gd\
        -lvtkIOLSDyna-7.0-gd\
        -lvtkIOMINC-7.0-gd\
        -lvtkIOMovie-7.0-gd\
        -lvtkIONetCDF-7.0-gd\
        -lvtkIOParallel-7.0-gd\
        -lvtkIOParallelXML-7.0-gd\
        -lvtkIOPLY-7.0-gd\
        -lvtkIOSQL-7.0-gd\
        -lvtkIOVideo-7.0-gd\
        -lvtkIOXML-7.0-gd\
        -lvtkIOXMLParser-7.0-gd\
        -lvtkjpeg-7.0-gd\
        -lvtkjsoncpp-7.0-gd\
        -lvtklibxml2-7.0-gd\
        -lvtkmetaio-7.0-gd\
        -lvtkNetCDF-7.0-gd\
        -lvtkNetCDF_cxx-7.0-gd\
        -lvtkoggtheora-7.0-gd\
        -lvtkParallelCore-7.0-gd\
        -lvtkpng-7.0-gd\
        -lvtkproj4-7.0-gd\
        -lvtkRenderingAnnotation-7.0-gd\
        -lvtkRenderingContext2D-7.0-gd\
        -lvtkRenderingContextOpenGL2-7.0-gd\
        -lvtkRenderingCore-7.0-gd\
        -lvtkRenderingFreeType-7.0-gd\
        -lvtkRenderingImage-7.0-gd\
        -lvtkRenderingLabel-7.0-gd\
        -lvtkRenderingLOD-7.0-gd\
        -lvtkRenderingOpenGL2-7.0-gd\
        -lvtkRenderingQt-7.0-gd\
        -lvtkRenderingVolume-7.0-gd\
        -lvtkRenderingVolumeOpenGL2-7.0-gd\
        -lvtksqlite-7.0-gd\
        -lvtksys-7.0-gd\
        -lvtktiff-7.0-gd\
        -lvtkverdict-7.0-gd\
        -lvtkViewsContext2D-7.0-gd\
        -lvtkViewsCore-7.0-gd\
        -lvtkViewsInfovis-7.0-gd\
        -lvtkViewsQt-7.0-gd\
        -lvtkzlib-7.0-gd

} else {

LIBS += -LF:/libQGLViewer/MSVC13_32bit_qt5.9.1/QGLViewer\
        -lQGLViewer2
LIBS += -lopengl32 \
    -lglu32

LIBS += -LF:/PCL/PCL1.8.0/lib\
        -lpcl_apps_release\
        -lpcl_common_release\
        -lpcl_features_release\
        -lpcl_filters_release\
        -lpcl_io_release\
        -lpcl_io_ply_release\
        -lpcl_kdtree_release\
        -lpcl_keypoints_release\
        -lpcl_ml_release\
        -lpcl_octree_release\
        -lpcl_outofcore_release\
        -lpcl_people_release\
        -lpcl_recognition_release\
        -lpcl_registration_release\
        -lpcl_sample_consensus_release\
        -lpcl_search_release\
        -lpcl_segmentation_release\
        -lpcl_simulation_release\
        -lpcl_stereo_release\
        -lpcl_surface_release\
        -lpcl_tracking_release\
        -lpcl_visualization_release\

LIBS += -LF:/PCL/PCL1.8.0/3rdParty/Boost/lib\
        -llibboost_atomic-vc120-mt-1_59\
        -llibboost_chrono-vc120-mt-1_59\
        -llibboost_container-vc120-mt-1_59\
        -llibboost_context-vc120-mt-1_59\
        -llibboost_coroutine-vc120-mt-1_59\
        -llibboost_date_time-vc120-mt-1_59\
        -llibboost_exception-vc120-mt-1_59\
        -llibboost_filesystem-vc120-mt-1_59\
        -llibboost_graph-vc120-mt-1_59\
        -llibboost_iostreams-vc120-mt-1_59\
        -llibboost_locale-vc120-mt-1_59\
        -llibboost_log-vc120-mt-1_59\
        -llibboost_log_setup-vc120-mt-1_59\
        -llibboost_math_c99-vc120-mt-1_59\
        -llibboost_math_c99f-vc120-mt-1_59\
        -llibboost_math_c99l-vc120-mt-1_59\
        -llibboost_math_tr1-vc120-mt-1_59\
        -llibboost_math_tr1f-vc120-mt-1_59\
        -llibboost_math_tr1l-vc120-mt-1_59\
        -llibboost_mpi-vc120-mt-1_59\
        -llibboost_prg_exec_monitor-vc120-mt-1_59\
        -llibboost_program_options-vc120-mt-1_59\
        -llibboost_random-vc120-mt-1_59\
        -llibboost_regex-vc120-mt-1_59\
        -llibboost_serialization-vc120-mt-1_59\
        -llibboost_signals-vc120-mt-1_59\
        -llibboost_system-vc120-mt-1_59\
        -llibboost_test_exec_monitor-vc120-mt-1_59\
        -llibboost_thread-vc120-mt-1_59\
        -llibboost_timer-vc120-mt-1_59\
        -llibboost_unit_test_framework-vc120-mt-1_59\
        -llibboost_wave-vc120-mt-1_59\
        -llibboost_wserialization-vc120-mt-1_59

LIBS += -LF:/PCL/PCL1.8.0/3rdParty/FLANN/lib\
        -lflann\
        -lflann_cpp_s\
        -lflann_s

LIBS += -LF:/PCL/PCL1.8.0/3rdParty/OpenNI2/Lib\
        -lOpenNI2

LIBS += -LF:/PCL/PCL1.8.0/3rdParty/Qhull/lib\
        -lqhull\
        -lqhullcpp\
        -lqhullstatic\
        -lqhullstatic_r\
        -lqhull_p\
        -lqhull_r

LIBS += -LF:/PCL/PCL1.8.0/3rdParty/VTK/lib\
        -lvtkalglib-7.0\
        -lvtkChartsCore-7.0\
        -lvtkCommonColor-7.0\
        -lvtkCommonComputationalGeometry-7.0\
        -lvtkCommonCore-7.0\
        -lvtkCommonDataModel-7.0\
        -lvtkCommonExecutionModel-7.0\
        -lvtkCommonMath-7.0\
        -lvtkCommonMisc-7.0\
        -lvtkCommonSystem-7.0\
        -lvtkCommonTransforms-7.0\
        -lvtkDICOMParser-7.0\
        -lvtkDomainsChemistry-7.0\
        -lvtkDomainsChemistryOpenGL2-7.0\
        -lvtkexoIIc-7.0\
        -lvtkexpat-7.0\
        -lvtkFiltersAMR-7.0\
        -lvtkFiltersCore-7.0\
        -lvtkFiltersExtraction-7.0\
        -lvtkFiltersFlowPaths-7.0\
        -lvtkFiltersGeneral-7.0\
        -lvtkFiltersGeneric-7.0\
        -lvtkFiltersGeometry-7.0\
        -lvtkFiltersHybrid-7.0\
        -lvtkFiltersHyperTree-7.0\
        -lvtkFiltersImaging-7.0\
        -lvtkFiltersModeling-7.0\
        -lvtkFiltersParallel-7.0\
        -lvtkFiltersParallelImaging-7.0\
        -lvtkFiltersProgrammable-7.0\
        -lvtkFiltersSelection-7.0\
        -lvtkFiltersSMP-7.0\
        -lvtkFiltersSources-7.0\
        -lvtkFiltersStatistics-7.0\
        -lvtkFiltersTexture-7.0\
        -lvtkFiltersVerdict-7.0\
        -lvtkfreetype-7.0\
        -lvtkGeovisCore-7.0\
        -lvtkglew-7.0\
        -lvtkGUISupportQt-7.0\
        -lvtkGUISupportQtSQL-7.0\
        -lvtkhdf5-7.0\
        -lvtkhdf5_hl-7.0\
        -lvtkImagingColor-7.0\
        -lvtkImagingCore-7.0\
        -lvtkImagingFourier-7.0\
        -lvtkImagingGeneral-7.0\
        -lvtkImagingHybrid-7.0\
        -lvtkImagingMath-7.0\
        -lvtkImagingMorphological-7.0\
        -lvtkImagingSources-7.0\
        -lvtkImagingStatistics-7.0\
        -lvtkImagingStencil-7.0\
        -lvtkInfovisCore-7.0\
        -lvtkInfovisLayout-7.0\
        -lvtkInteractionImage-7.0\
        -lvtkInteractionStyle-7.0\
        -lvtkInteractionWidgets-7.0\
        -lvtkIOAMR-7.0\
        -lvtkIOCore-7.0\
        -lvtkIOEnSight-7.0\
        -lvtkIOExodus-7.0\
        -lvtkIOExport-7.0\
        -lvtkIOGeometry-7.0\
        -lvtkIOImage-7.0\
        -lvtkIOImport-7.0\
        -lvtkIOInfovis-7.0\
        -lvtkIOLegacy-7.0\
        -lvtkIOLSDyna-7.0\
        -lvtkIOMINC-7.0\
        -lvtkIOMovie-7.0\
        -lvtkIONetCDF-7.0\
        -lvtkIOParallel-7.0\
        -lvtkIOParallelXML-7.0\
        -lvtkIOPLY-7.0\
        -lvtkIOSQL-7.0\
        -lvtkIOVideo-7.0\
        -lvtkIOXML-7.0\
        -lvtkIOXMLParser-7.0\
        -lvtkjpeg-7.0\
        -lvtkjsoncpp-7.0\
        -lvtklibxml2-7.0\
        -lvtkmetaio-7.0\
        -lvtkNetCDF-7.0\
        -lvtkNetCDF_cxx-7.0\
        -lvtkoggtheora-7.0\
        -lvtkParallelCore-7.0\
        -lvtkpng-7.0\
        -lvtkproj4-7.0\
        -lvtkRenderingAnnotation-7.0\
        -lvtkRenderingContext2D-7.0\
        -lvtkRenderingContextOpenGL2-7.0\
        -lvtkRenderingCore-7.0\
        -lvtkRenderingFreeType-7.0\
        -lvtkRenderingImage-7.0\
        -lvtkRenderingLabel-7.0\
        -lvtkRenderingLOD-7.0\
        -lvtkRenderingOpenGL2-7.0\
        -lvtkRenderingQt-7.0\
        -lvtkRenderingVolume-7.0\
        -lvtkRenderingVolumeOpenGL2-7.0\
        -lvtksqlite-7.0\
        -lvtksys-7.0\
        -lvtktiff-7.0\
        -lvtkverdict-7.0\
        -lvtkViewsContext2D-7.0\
        -lvtkViewsCore-7.0\
        -lvtkViewsInfovis-7.0\
        -lvtkViewsQt-7.0\
        -lvtkzlib-7.0
}

RESOURCES += \
    src.qrc \
    image/axis.qrc
CONFIG += C++11
RC_FILE = tunnel_ico.rc
