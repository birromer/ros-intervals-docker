cmake_minimum_required(VERSION 2.8...3.16)
project(robot_control)


find_package(Qt5 COMPONENTS Core Gui Widgets Quick Network QuickWidgets Positioning)

find_package(Qt5QuickCompiler)

find_package(PROJ QUIET)
if(PROJ VERSION_LESS "4.0")
        find_package(PROJ4 REQUIRED)
        set(PROJ_LIBRARIES ${PROJ4_LIBRARIES})
endif()

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)


add_executable(robotcontrol main.cpp robotcontrol.cpp protocol.cpp serial/serial.cc serial/impl/unix.cc serial/impl/win.cc serial/impl/list_ports/list_ports_linux.cc serial/impl/list_ports/list_ports_win.cc)
qt5_use_modules(robotcontrol Core Gui Widgets Quick Network QuickWidgets Positioning)
target_link_libraries(robotcontrol ${PROJ_LIBRARIES})
if(WIN32)
        target_link_libraries(robotcontrol Setupapi) # For serial.
endif()
