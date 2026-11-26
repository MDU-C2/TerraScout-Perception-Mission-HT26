# This file was automatically generated for projects
# without default 'CMakeLists.txt' file.

if(RC_MONITOR_BUILD)
    set(app_sources sbus.cpp rc_monitor.cpp)
elseif(BMS_MONITOR_BUILD)
    set(app_sources can.cpp bms_interface.cpp bms_monitor.cpp)
else()
    FILE(GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/src/*.*)
    list(FILTER app_sources EXCLUDE REGEX "/bms_monitor\\.cpp$")
    list(FILTER app_sources EXCLUDE REGEX "/rc_monitor\\.cpp$")
endif()

idf_component_register(SRCS ${app_sources} INCLUDE_DIRS "../include")
