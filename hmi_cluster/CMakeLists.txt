cmake_minimum_required(VERSION 3.16)
project(qnx_hmi_cluster LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Qt 6 の MOC (Meta-Object Compiler) を自動実行
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

find_package(Qt6 REQUIRED COMPONENTS Core Gui Widgets OpenGLWidgets Network)

add_executable(qnx_hmi_cluster
    src/main.cpp
    src/glwidget.h
    src/glwidget.cpp
)

target_link_libraries(qnx_hmi_cluster PRIVATE
    Qt6::Core
    Qt6::Gui
    Qt6::Widgets
    Qt6::OpenGLWidgets
    Qt6::Network
)
