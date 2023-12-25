cmake_minimum_required(VERSION 3.5)

project(RoadMarking LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Qt5Core)
find_package(laslib CONFIG REQUIRED)
find_package(spdlog CONFIG REQUIRED)

add_executable(RoadMarking
  main.cpp
)
target_link_libraries(RoadMarking Qt5::Core LASlib spdlog::spdlog_header_only)
