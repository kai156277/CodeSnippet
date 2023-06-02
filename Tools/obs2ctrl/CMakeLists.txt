# CMake for Qt
set(QT Core Gui Widgets)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
find_package(Qt5 REQUIRED ${QT})

find_package(fmt REQUIRED)

# 将源代码添加到此项目的可执行文件。
add_executable(obs2ctrl "obs2ctrl.cpp" "obs2ctrl.h")

target_link_libraries(obs2ctrl PRIVATE
	fmt::fmt-header-only
	Qt::Core
	Qt::Gui
	Qt::Widgets
	)
