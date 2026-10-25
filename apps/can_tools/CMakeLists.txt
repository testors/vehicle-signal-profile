cmake_minimum_required(VERSION 3.10)
project(can_tools C OBJC)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Compiler flags
if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(-Wall -Wextra -O2)
endif()

# Common include path
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/common)

# Discovery core library
add_library(can_discover_core STATIC
    common/can_discover.c
)
target_include_directories(can_discover_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/common
)
target_link_libraries(can_discover_core PUBLIC m)

# Shared transport / protocol sources
set(COMMON_SOURCES
    common/elm327.c
    common/stn2255.c
)

set(OBD_ADAPTER_SOURCES
    common/obd_adapter.m
)

# --- can_reader ---
add_executable(can_reader
    can_reader/main.m
    ${COMMON_SOURCES}
)
target_link_libraries(can_reader
    can_discover_core
    "-framework CoreBluetooth"
    "-framework IOBluetooth"
    "-framework Foundation"
    m
)

# --- can_discover ---
add_executable(can_discover
    can_discover/main.m
    ${COMMON_SOURCES}
)
target_link_libraries(can_discover
    can_discover_core
    "-framework CoreBluetooth"
    "-framework IOBluetooth"
    "-framework Foundation"
    m
)

# --- vsp_probe ---
add_executable(vsp_probe
    vsp_probe/main.m
    ${COMMON_SOURCES}
    ${OBD_ADAPTER_SOURCES}
)
target_link_libraries(vsp_probe
    "-framework CoreBluetooth"
    "-framework IOBluetooth"
    "-framework Foundation"
    m
)

# --- discover_replay (offline re-analysis of a capture session; plain C) ---
add_executable(discover_replay
    tools/discover_replay.c
)
target_link_libraries(discover_replay
    can_discover_core
    m
)

# --- discover_regression (synthetic engine tests; run with ctest) ---
enable_testing()
find_package(Threads REQUIRED)
add_executable(discover_regression
    tests/discover_regression.c
)
target_link_libraries(discover_regression
    can_discover_core
    Threads::Threads
    m
)
add_test(NAME discover_regression COMMAND discover_regression)

message(STATUS "CAN Tools Build Configuration")
message(STATUS "  Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "  C Compiler: ${CMAKE_C_COMPILER}")
message(STATUS "  Targets: can_discover_core, can_reader, can_discover, vsp_probe")
