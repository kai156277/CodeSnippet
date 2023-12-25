cmake_minimum_required(VERSION 3.5)

project(PCLTest LANGUAGES CXX)

set(QT Core Widgets Gui)
find_package(PCL CONFIG REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)
find_package(fmt CONFIG REQUIRED)
find_package(Qt5 REQUIRED ${QT})
find_package(spdlog REQUIRED)
find_package(laslib CONFIG REQUIRED)

# Eigen + fmt 9.1.0 需要此配置
add_definitions(-DFMT_DEPRECATED_OSTREAM)
add_definitions(-DSPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_TRACE)
add_definitions(-DWIM_CAL_LIB_API)


set(SOURCES
        "main.cpp")

set(PUBLIC_HEADERS )

set(PRIVATE_HEADERS )


add_executable(${PROJECT_NAME} ${BUILD_TYPE} ${SOURCES} ${PUBLIC_HEADERS} ${PRIVATE_HEADERS} )

target_link_libraries(${PROJECT_NAME}
        ${PCL_LIBRARIES}
        LASlib
        Eigen3::Eigen
        fmt::fmt-header-only
        spdlog::spdlog_header_only
        Qt5::Core
        Qt5::Gui
        Qt5::Widgets
        )
