﻿# CMakeList.txt: process2cal 的 CMake 项目，在此处包括源代码并定义
# 项目特定的逻辑。
#
cmake_minimum_required (VERSION 3.8)

# CMake for Qt
set(QT Core Gui Widgets)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
find_package(Qt5 REQUIRED ${QT})

find_package(fmt REQUIRED)

# 将源代码添加到此项目的可执行文件。
add_executable (process2cal "process2cal.cpp" "process2cal.h")

message("cmake binary dir" ${CMAKE_BINARY_DIR})

target_link_libraries(process2cal PRIVATE
	fmt::fmt-header-only
	Qt::Core
	Qt::Gui
	Qt::Widgets
	)

add_executable (st_process2cal "STfileProcess2cal.cpp")

target_link_libraries(st_process2cal PRIVATE
	Qt::Core
	Qt::Gui
	Qt::Widgets
	)

set_target_properties(process2cal st_process2cal
		PROPERTIES
		ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
		LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
		RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
		)

# TODO: 如有需要，请添加测试并安装目标。
