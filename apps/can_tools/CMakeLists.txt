cmake_minimum_required(VERSION 3.12)
project(can_tools C OBJC)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Compiler flags
if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(-Wall -Wextra -O2)
endif()

# Common include path
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/common)

# This repository's VSP profiles (apps/can_tools/../../profiles)
get_filename_component(VSP_PROFILES_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../profiles ABSOLUTE)

# --- VSP corpus prior table: generated from the profiles at build time, not committed ---
find_program(PYTHON3_EXECUTABLE python3)
if(NOT PYTHON3_EXECUTABLE)
    message(FATAL_ERROR "python3 is required: the build generates can_discover_priors.h "
                        "from ${VSP_PROFILES_DIR} with tools/gen_can_discover_priors.py")
endif()
file(GLOB_RECURSE VSP_PROFILE_FILES CONFIGURE_DEPENDS ${VSP_PROFILES_DIR}/*.vsp)
set(PRIORS_DIR ${CMAKE_CURRENT_BINARY_DIR}/generated)
set(PRIORS_HEADER ${PRIORS_DIR}/can_discover_priors.h)
# Rewritten only when the file list changes, so a removed profile also regenerates the header.
string(REPLACE ";" "\n" VSP_PROFILE_LIST "${VSP_PROFILE_FILES}")
file(GENERATE OUTPUT ${PRIORS_DIR}/profiles.list CONTENT "${VSP_PROFILE_LIST}\n")
add_custom_command(
    OUTPUT ${PRIORS_HEADER}
    COMMAND ${CMAKE_COMMAND} -E make_directory ${PRIORS_DIR}
    COMMAND ${PYTHON3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tools/gen_can_discover_priors.py
            ${VSP_PROFILES_DIR} -o ${PRIORS_HEADER}
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/tools/gen_can_discover_priors.py
            ${PRIORS_DIR}/profiles.list ${VSP_PROFILE_FILES}
    COMMENT "Generating can_discover_priors.h from ${VSP_PROFILES_DIR}"
    VERBATIM
)

# Discovery core library
add_library(can_discover_core STATIC
    common/can_discover.c
    ${PRIORS_HEADER}
)
target_include_directories(can_discover_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/common
)
target_include_directories(can_discover_core PRIVATE ${PRIORS_DIR})
target_link_libraries(can_discover_core PUBLIC m)

# Frame sources (live/replay interface), capture TSV reader, CLI helpers (plain C)
add_library(can_frames STATIC
    common/frame_tsv.c
    common/frame_source.c
    common/cli_util.c
)
target_include_directories(can_frames PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/common
)

# PCAN-USB through UV Software's PCBUSB library, receive only (plain C).
# The library is loaded with dlopen at run time: no link-time dependency, the
# build and every other command work without it installed.
add_library(pcan_receive STATIC
    common/pcan.c
    common/frame_source_pcan.c
)
target_include_directories(pcan_receive PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/common
)
target_link_libraries(pcan_receive PUBLIC can_frames)

# ELM327 / STN2255 command and response handling (plain C)
add_library(obd_protocol STATIC
    common/elm327.c
    common/stn2255.c
)
target_include_directories(obd_protocol PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/common
)

# --- Objective-C shared code (Foundation, manual retain/release) ---

# BLE/Classic adapter, the scan/connect/init path, live frame source
add_library(obd_transport STATIC
    common/obd_adapter.m
    common/obd_connect.m
    common/obd_poll.m
    common/frame_source_live.m
)
target_link_libraries(obd_transport PUBLIC
    obd_protocol
    can_frames
    "-framework CoreBluetooth"
    "-framework IOBluetooth"
    "-framework Foundation"
)

# Guided discover session + interactive CLI driver
add_library(discover_session STATIC
    common/discover_session.m
)
target_link_libraries(discover_session PUBLIC
    can_discover_core
    can_frames
    "-framework Foundation"
)

# VSP profile / signal decoder
add_library(vsp_engine STATIC
    common/vsp_engine.m
)
target_link_libraries(vsp_engine PUBLIC
    "-framework Foundation"
    m
)

# --- TUI core: model, rendering into a cell grid, replay loading, recorder, stderr capture (plain C) ---
add_library(tui_core STATIC
    can_tools/tui/tui_grid.c
    can_tools/tui/tui_model.c
    can_tools/tui/tui_render.c
    can_tools/tui/tui_screens.c
    can_tools/tui/tui_gauges.c
    can_tools/tui/tui_capture.c
    can_tools/tui/tui_replay.c
    can_tools/tui/tui_recorder.c
)
target_include_directories(tui_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/can_tools/tui
)
target_link_libraries(tui_core PUBLIC
    can_discover_core
    can_frames
    m
)

# --- can_tools: the single binary ---
# The sources build once as an object library: can_tools links them with the
# real Bluetooth edges, can_tools_fake (tests only) with tests/fake/fake_adapter.m.
# tui_main.m compiles per target: only can_tools_fake gets CAN_TOOLS_TEST_HOOKS
# (the headless live test hook); the real binary has no such path.
add_library(can_tools_objs OBJECT
    can_tools/main.m
    can_tools/cli_common.m
    can_tools/cli_pcan.c
    can_tools/interactive.m
    can_tools/cmd_scan.m
    can_tools/cmd_detect.m
    can_tools/cmd_monitor.m
    can_tools/cmd_query.m
    can_tools/cmd_discover.m
    can_tools/cmd_probe.m
    can_tools/cmd_replay.c
    can_tools/cmd_p992.m
    can_tools/tui/tui_live.m
    can_tools/tui/tui_app.c
    can_tools/tui/tui_profiles.c
    can_tools/tui/tui_term.c
)
target_include_directories(can_tools_objs PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/can_tools
    ${CMAKE_CURRENT_SOURCE_DIR}/can_tools/tui
)
# The TUI profile picker looks here when VSP_PROFILES_DIR and ./profiles are absent.
target_compile_definitions(can_tools_objs PRIVATE CAN_TOOLS_PROFILES_DIR="${VSP_PROFILES_DIR}")
target_link_libraries(can_tools_objs PUBLIC
    discover_session   # libm via can_discover_core
    vsp_engine
    obd_transport
    pcan_receive
    tui_core
    ncurses
)

add_executable(can_tools $<TARGET_OBJECTS:can_tools_objs> can_tools/tui/tui_main.m)
target_include_directories(can_tools PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/can_tools
    ${CMAKE_CURRENT_SOURCE_DIR}/can_tools/tui
)
target_link_libraries(can_tools can_tools_objs)

# Old command names keep working: argv[0] selects the tool's command line.
foreach(alias can_reader can_discover vsp_probe discover_replay)
    add_custom_command(TARGET can_tools POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E rm -f $<TARGET_FILE_DIR:can_tools>/${alias}
        COMMAND ${CMAKE_COMMAND} -E create_symlink can_tools $<TARGET_FILE_DIR:can_tools>/${alias}
        VERBATIM
    )
endforeach()

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

# --- vsp_engine_test (decoder rules on an embedded VSP; no hardware) ---
add_executable(vsp_engine_test
    tests/vsp_engine_test.c
)
target_link_libraries(vsp_engine_test
    vsp_engine
)
add_test(NAME vsp_engine_test COMMAND vsp_engine_test)

# --- frame_source_test (TSV replay source; no hardware) ---
add_executable(frame_source_test
    tests/frame_source_test.c
)
target_link_libraries(frame_source_test
    can_frames
)
add_test(NAME frame_source_test COMMAND frame_source_test)

# --- obd_adapter_lifecycle_test (adapter create/use/free across autorelease pools) ---
add_executable(obd_adapter_lifecycle_test
    tests/obd_adapter_lifecycle_test.m
)
target_link_libraries(obd_adapter_lifecycle_test
    obd_transport
)
add_test(NAME obd_adapter_lifecycle_test COMMAND obd_adapter_lifecycle_test)
# Zombies turn a message to a freed object into a deterministic crash.
set_tests_properties(obd_adapter_lifecycle_test PROPERTIES ENVIRONMENT "NSZombieEnabled=YES")

# --- tui_model_test (TUI model, grid, replay loading, recorder; plain C) ---
add_executable(tui_model_test
    tests/tui_model_test.c
)
target_link_libraries(tui_model_test
    tui_core
)
add_test(NAME tui_model_test COMMAND tui_model_test)

# --- tui_gauge_test (gauges panel, braille timeline, colors; plain C) ---
add_executable(tui_gauge_test
    tests/tui_gauge_test.c
)
target_link_libraries(tui_gauge_test
    tui_core
)
add_test(NAME tui_gauge_test COMMAND tui_gauge_test)

# --- tui_profiles_test (profile index for the picker; fixtures only) ---
add_executable(tui_profiles_test
    tests/tui_profiles_test.c
    can_tools/tui/tui_profiles.c
)
target_include_directories(tui_profiles_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/can_tools/tui)
target_link_libraries(tui_profiles_test vsp_engine)
add_test(NAME tui_profiles_test COMMAND tui_profiles_test ${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/tui)

# --- tui_term_rep_test (ncurses backend writes every character under a rep=%p1%c terminfo; pty, no terminal) ---
add_executable(tui_term_rep_test
    tests/tui_term_rep_test.c
    can_tools/tui/tui_term.c
)
target_include_directories(tui_term_rep_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/can_tools/tui)
target_link_libraries(tui_term_rep_test tui_core ncurses)
add_test(NAME tui_term_rep_test COMMAND tui_term_rep_test)

# --- tui_snapshot_golden (headless replay snapshots vs tests/golden/tui; no terminal, no adapter) ---
add_test(NAME tui_snapshot_golden
    COMMAND sh ${CMAKE_CURRENT_SOURCE_DIR}/tests/golden/tui/snapshots.sh check $<TARGET_FILE:can_tools>
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)

# --- can_tools_fake: can_tools with a fake adapter (no Bluetooth object is created)
#     and a fake PCAN library (tests/fake/fake_pcan.c: the real PCBUSB is never loaded) ---
add_executable(can_tools_fake $<TARGET_OBJECTS:can_tools_objs> can_tools/tui/tui_main.m
    tests/fake/fake_adapter.m tests/fake/fake_pcan.c)
target_include_directories(can_tools_fake PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/can_tools
    ${CMAKE_CURRENT_SOURCE_DIR}/can_tools/tui
)
target_compile_definitions(can_tools_fake PRIVATE CAN_TOOLS_TEST_HOOKS=1)
target_link_libraries(can_tools_fake can_tools_objs)

# --- tui_no_test_hooks (the real can_tools rejects the headless hook during option checks) ---
add_test(NAME tui_no_test_hooks
    COMMAND sh ${CMAKE_CURRENT_SOURCE_DIR}/tests/tui_hooks_check.sh $<TARGET_FILE:can_tools>
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)

# --- tui_fake_safety (live TUI path through the fake adapter: exact AT command logs) ---
add_test(NAME tui_fake_safety
    COMMAND sh ${CMAKE_CURRENT_SOURCE_DIR}/tests/tui_fake/run.sh check $<TARGET_FILE:can_tools_fake>
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)

# --- pcan_test (listen-only open sequence, auto bitrate, clock mapping, PCAN source,
#     6-column TSV; a fake pcan_api_t, no library, no hardware) ---
add_executable(pcan_test
    tests/pcan_test.c
)
target_link_libraries(pcan_test
    pcan_receive
)
add_test(NAME pcan_test COMMAND pcan_test)

# --- pcan_fake_safety (CLI refusals, no CAN_Write in the binaries, headless TUI over a fake PCAN) ---
add_test(NAME pcan_fake_safety
    COMMAND sh ${CMAKE_CURRENT_SOURCE_DIR}/tests/pcan_fake/run.sh $<TARGET_FILE:can_tools_fake> $<TARGET_FILE:can_tools>
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)

# --- discover_session_test (session layer fed by replay sources; no hardware) ---
add_executable(discover_session_test
    tests/discover_session_test.c
)
target_link_libraries(discover_session_test
    discover_session
)
add_test(NAME discover_session_test COMMAND discover_session_test)

message(STATUS "CAN Tools Build Configuration")
message(STATUS "  Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "  C Compiler: ${CMAKE_C_COMPILER}")
message(STATUS "  Targets: can_tools (+ aliases can_reader, can_discover, vsp_probe, discover_replay), can_tools_fake (tests),")
message(STATUS "           can_discover_core, can_frames, pcan_receive, obd_protocol, obd_transport, discover_session, vsp_engine")
