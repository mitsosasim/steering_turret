cmake_minimum_required(VERSION 3.16)

project(STC C CXX)

include(CTest)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(BUILD_DIAG_GUI "Build Qt-based diagnostics GUI" ON)

option(USE_SIMULATOR "Use software motor simulator" ON)

if(USE_SIMULATOR)
  add_library(simulator STATIC
    dev/src/simulation/sim_backend.cpp
    dev/src/simulation/sim_api.cpp
    dev/stub_ST/MotorModel.cpp
  )
  target_include_directories(simulator PUBLIC
    ${PROJECT_SOURCE_DIR}/dev/include
    ${PROJECT_SOURCE_DIR}/dev/src/simulation
    ${PROJECT_SOURCE_DIR}/dev/stub_ST
  )
endif()

add_executable(stc_server
  dev/src/stc_server.c
  dev/src/stc_log.c
  dev/src/stc_icd.c
  dev/src/stc_model.c
  dev/src/stc_config.c
  dev/src/stc_axis_cfg_map.c
  dev/src/pd4c.c
  dev/src/stc_actions.c
  dev/src/stc_limits.c
  dev/src/stc_state.c
  dev/src/stc_faults.c
  dev/src/stc_diag.c
  dev/src/motion_backend_router.c

  # Backend families: simulator / pd4c-can / myactuator
  dev/src/can_backend.c      # pd4c-can shared backend dispatch implementation
  dev/src/pd4c_backend.c     # pd4c-can backend family
  dev/src/myact_backend.c    # myactuator backend family
)

if(NOT USE_SIMULATOR)
  target_sources(stc_server PRIVATE
    # Transport/runtime dependencies for CAN-backed families.
    dev/src/x8_25.c
    dev/src/myact_socketcan.c
  )
endif()

add_executable(icd_tx_tcp
  dev/tools/icd_tx_tcp.c
  dev/src/stc_icd.c
)

add_executable(tlm_rx_udp
  dev/tools/tlm_rx_udp.c
  dev/src/stc_icd.c
)

target_include_directories(stc_server PRIVATE ${PROJECT_SOURCE_DIR}/dev/include)
if(USE_SIMULATOR)
  target_include_directories(stc_server PRIVATE ${PROJECT_SOURCE_DIR}/dev/src/simulation)
endif()
target_include_directories(icd_tx_tcp PRIVATE ${PROJECT_SOURCE_DIR}/dev/include)
target_include_directories(tlm_rx_udp PRIVATE ${PROJECT_SOURCE_DIR}/dev/include)

target_compile_options(stc_server PRIVATE -Wall -Wextra)
target_compile_options(icd_tx_tcp PRIVATE -Wall -Wextra)
target_compile_options(tlm_rx_udp PRIVATE -Wall -Wextra)

target_link_libraries(stc_server PRIVATE pthread m)

if(USE_SIMULATOR)
  target_compile_definitions(stc_server PRIVATE STC_USE_SIMULATOR=1)
else()
  target_compile_definitions(stc_server PRIVATE STC_USE_SIMULATOR=0)
endif()

if(USE_SIMULATOR)
  target_compile_definitions(stc_server PRIVATE SIMULATION)
  target_link_libraries(stc_server PRIVATE simulator)
else()
  target_compile_definitions(stc_server PRIVATE
    _DEFAULT_SOURCE
    _POSIX_C_SOURCE=200809L
  )
endif()

add_executable(test_actions
  dev/tests/test_actions.c
  dev/src/stc_actions.c
  dev/src/stc_limits.c
)

target_include_directories(test_actions PRIVATE ${PROJECT_SOURCE_DIR}/dev/include)
target_compile_options(test_actions PRIVATE -Wall -Wextra)

add_executable(stc_state_test
  dev/tests/stc_state_test.c
  dev/src/stc_model.c
  dev/src/stc_state.c
  dev/src/stc_faults.c
  dev/src/stc_diag.c
  dev/src/stc_log.c
  dev/src/motion_backend_router.c
  dev/src/can_backend.c
  dev/src/pd4c_backend.c
  dev/src/myact_backend.c
  dev/src/stc_limits.c
)

target_include_directories(stc_state_test PRIVATE ${PROJECT_SOURCE_DIR}/dev/include)
target_compile_options(stc_state_test PRIVATE -Wall -Wextra)
target_link_libraries(stc_state_test PRIVATE m)
if(USE_SIMULATOR)
  target_compile_definitions(stc_state_test PRIVATE SIMULATION)
  target_link_libraries(stc_state_test PRIVATE simulator)
else()
  target_compile_definitions(stc_state_test PRIVATE
    _DEFAULT_SOURCE
    _POSIX_C_SOURCE=200809L
  )
  target_sources(stc_state_test PRIVATE
    dev/src/pd4c.c
    dev/src/x8_25.c
    dev/src/myact_socketcan.c
  )
endif()

add_executable(stc_diag_test
  dev/tests/test_diag_codes.c
  dev/src/stc_model.c
  dev/src/stc_diag.c
  dev/src/stc_log.c
  dev/src/motion_backend_router.c
  dev/src/can_backend.c
  dev/src/pd4c_backend.c
  dev/src/myact_backend.c
  dev/src/stc_axis_cfg_map.c
  dev/src/stc_limits.c
)


add_executable(test_diag_codes
  dev/tests/test_diag_codes.c
  dev/src/stc_model.c
  dev/src/stc_diag.c
  dev/src/stc_log.c
  dev/src/motion_backend_router.c
  dev/src/pd4c_backend.c
  dev/src/myact_backend.c
  dev/src/stc_axis_cfg_map.c
  dev/src/stc_limits.c
)

target_include_directories(test_diag_codes PRIVATE ${PROJECT_SOURCE_DIR}/dev/include)
target_compile_options(test_diag_codes PRIVATE -Wall -Wextra)
target_link_libraries(test_diag_codes PRIVATE m)
if(USE_SIMULATOR)
  target_compile_definitions(test_diag_codes PRIVATE SIMULATION)
  target_link_libraries(test_diag_codes PRIVATE simulator)
else()
  target_compile_definitions(test_diag_codes PRIVATE
    _DEFAULT_SOURCE
    _POSIX_C_SOURCE=200809L
  )
  target_sources(test_diag_codes PRIVATE
    dev/src/can_backend.c
    dev/src/pd4c.c
    dev/src/x8_25.c
    dev/src/myact_socketcan.c
  )
endif()

add_executable(test_diag_state_integration
  dev/tests/test_diag_state_integration.c
  dev/src/stc_model.c
  dev/src/stc_state.c
  dev/src/stc_diag.c
  dev/src/stc_log.c
  dev/src/motion_backend_router.c
  dev/src/pd4c_backend.c
  dev/src/myact_backend.c
  dev/src/stc_limits.c
)

add_executable(test_icd_codec
  dev/tests/test_icd_codec.c
  dev/src/stc_icd.c
)

target_include_directories(test_icd_codec PRIVATE ${PROJECT_SOURCE_DIR}/dev/include)
target_compile_options(test_icd_codec PRIVATE -Wall -Wextra)

add_executable(test_myact_socketcan_ids
  dev/tests/test_myact_socketcan_ids.c
  dev/src/x8_25.c
)
target_include_directories(test_myact_socketcan_ids PRIVATE ${PROJECT_SOURCE_DIR}/dev/include)
target_compile_options(test_myact_socketcan_ids PRIVATE -Wall -Wextra)
target_compile_definitions(test_myact_socketcan_ids PRIVATE _DEFAULT_SOURCE)
target_link_libraries(test_myact_socketcan_ids PRIVATE m)

add_executable(x8_25_test_can
  dev/tests/x8_25_test.c
  dev/src/x8_25.c
  dev/src/myact_socketcan.c
)
target_include_directories(x8_25_test_can PRIVATE ${PROJECT_SOURCE_DIR}/dev/include)
target_compile_options(x8_25_test_can PRIVATE -Wall -Wextra)
target_compile_definitions(x8_25_test_can PRIVATE _DEFAULT_SOURCE)
target_link_libraries(x8_25_test_can PRIVATE m)

target_include_directories(stc_diag_test PRIVATE ${PROJECT_SOURCE_DIR}/dev/include)
target_compile_options(stc_diag_test PRIVATE -Wall -Wextra)
target_link_libraries(stc_diag_test PRIVATE m)
if(USE_SIMULATOR)
  target_compile_definitions(stc_diag_test PRIVATE SIMULATION)
  target_link_libraries(stc_diag_test PRIVATE simulator)
else()
  target_compile_definitions(stc_diag_test PRIVATE
    _DEFAULT_SOURCE
    _POSIX_C_SOURCE=200809L
  )
  target_sources(stc_diag_test PRIVATE
    dev/src/pd4c.c
    dev/src/x8_25.c
    dev/src/myact_socketcan.c
  )
endif()
target_include_directories(test_diag_state_integration PRIVATE ${PROJECT_SOURCE_DIR}/dev/include)
target_compile_options(test_diag_state_integration PRIVATE -Wall -Wextra)
target_link_libraries(test_diag_state_integration PRIVATE m)
if(USE_SIMULATOR)
  target_compile_definitions(test_diag_state_integration PRIVATE SIMULATION)
  target_link_libraries(test_diag_state_integration PRIVATE simulator)
else()
  target_compile_definitions(test_diag_state_integration PRIVATE
    _DEFAULT_SOURCE
    _POSIX_C_SOURCE=200809L
  )
  target_sources(test_diag_state_integration PRIVATE
    dev/src/can_backend.c
    dev/src/pd4c.c
    dev/src/x8_25.c
    dev/src/myact_socketcan.c
  )
endif()



add_executable(test_axis_cfg_map
  dev/tests/test_axis_cfg_map.c
  dev/src/stc_axis_cfg_map.c
  dev/src/stc_limits.c
)

target_include_directories(test_axis_cfg_map PRIVATE ${PROJECT_SOURCE_DIR}/dev/include)
target_compile_options(test_axis_cfg_map PRIVATE -Wall -Wextra)

add_executable(stc_config_test
  dev/tests/stc_config_test.c
  dev/src/stc_config.c
  dev/src/stc_log.c
  dev/src/stc_model.c
)

target_include_directories(stc_config_test PRIVATE ${PROJECT_SOURCE_DIR}/dev/include)
target_compile_options(stc_config_test PRIVATE -Wall -Wextra)
target_link_libraries(stc_config_test PRIVATE m)

add_executable(test_log_concurrency
  dev/tests/test_log_concurrency.c
  dev/src/stc_log.c
)

target_include_directories(test_log_concurrency PRIVATE ${PROJECT_SOURCE_DIR}/dev/include)
target_compile_options(test_log_concurrency PRIVATE -Wall -Wextra)
target_link_libraries(test_log_concurrency PRIVATE pthread)
add_executable(test_can_worker_concurrency
  dev/tests/test_can_worker_concurrency.c
  dev/src/stc_log.c
  dev/src/stc_icd.c
  dev/src/stc_model.c
  dev/src/stc_config.c
  dev/src/pd4c.c
  dev/src/stc_actions.c
  dev/src/stc_limits.c
  dev/src/stc_state.c
  dev/src/stc_faults.c
  dev/src/stc_diag.c
  dev/src/motion_backend_router.c
  dev/src/stc_axis_cfg_map.c
  dev/src/pd4c_backend.c
  dev/src/myact_backend.c
)

target_include_directories(test_can_worker_concurrency PRIVATE ${PROJECT_SOURCE_DIR}/dev/include)
if(USE_SIMULATOR)
  target_include_directories(test_can_worker_concurrency PRIVATE ${PROJECT_SOURCE_DIR}/dev/src/simulation ${PROJECT_SOURCE_DIR}/dev/stub_ST)
endif()
target_compile_options(test_can_worker_concurrency PRIVATE -Wall -Wextra -Wno-unused-function)
target_link_libraries(test_can_worker_concurrency PRIVATE pthread m)
if(USE_SIMULATOR)
  target_compile_definitions(test_can_worker_concurrency PRIVATE SIMULATION)
  target_link_libraries(test_can_worker_concurrency PRIVATE simulator)
else()
  target_compile_definitions(test_can_worker_concurrency PRIVATE
    _DEFAULT_SOURCE
    _POSIX_C_SOURCE=200809L
  )
  target_sources(test_can_worker_concurrency PRIVATE
    dev/src/can_backend.c
    dev/src/x8_25.c
    dev/src/myact_socketcan.c
  )
endif()

add_executable(test_can_worker_concurrency_tsan
  dev/tests/test_can_worker_concurrency.c
  dev/src/stc_log.c
  dev/src/stc_icd.c
  dev/src/stc_model.c
  dev/src/stc_config.c
  dev/src/pd4c.c
  dev/src/stc_actions.c
  dev/src/stc_limits.c
  dev/src/stc_state.c
  dev/src/stc_faults.c
  dev/src/stc_diag.c
  dev/src/motion_backend_router.c
  dev/src/stc_axis_cfg_map.c
  dev/src/pd4c_backend.c
  dev/src/myact_backend.c
)

target_include_directories(test_can_worker_concurrency_tsan PRIVATE ${PROJECT_SOURCE_DIR}/dev/include)
if(USE_SIMULATOR)
  target_include_directories(test_can_worker_concurrency_tsan PRIVATE ${PROJECT_SOURCE_DIR}/dev/src/simulation ${PROJECT_SOURCE_DIR}/dev/stub_ST)
endif()
target_compile_options(test_can_worker_concurrency_tsan PRIVATE -Wall -Wextra -Wno-unused-function -fsanitize=thread -fno-omit-frame-pointer)
target_link_options(test_can_worker_concurrency_tsan PRIVATE -fsanitize=thread -fno-omit-frame-pointer)
target_link_libraries(test_can_worker_concurrency_tsan PRIVATE pthread m)
if(USE_SIMULATOR)
  target_compile_definitions(test_can_worker_concurrency_tsan PRIVATE SIMULATION)
  target_link_libraries(test_can_worker_concurrency_tsan PRIVATE simulator)
else()
  target_compile_definitions(test_can_worker_concurrency_tsan PRIVATE
    _DEFAULT_SOURCE
    _POSIX_C_SOURCE=200809L
  )
  target_sources(test_can_worker_concurrency_tsan PRIVATE
    dev/src/can_backend.c
    dev/src/x8_25.c
    dev/src/myact_socketcan.c
  )
endif()


add_executable(test_motion_backend_router
  dev/tests/test_motion_backend_router.c
  dev/src/motion_backend_router.c
  dev/src/stc_log.c
)
target_include_directories(test_motion_backend_router PRIVATE ${PROJECT_SOURCE_DIR}/dev/include)
target_compile_options(test_motion_backend_router PRIVATE -Wall -Wextra)
if(USE_SIMULATOR)
  target_compile_definitions(test_motion_backend_router PRIVATE SIMULATION)
else()
  target_compile_definitions(test_motion_backend_router PRIVATE
    _DEFAULT_SOURCE
    _POSIX_C_SOURCE=200809L
  )
endif()

if(BUILD_DIAG_GUI)
  find_package(Qt6 COMPONENTS Core Gui Widgets Network QUIET)
  if(Qt6_FOUND)
    add_executable(diag_gui_qt
      dev/tools/diag_gui_qt.cpp
      dev/src/stc_icd.c
    )
    target_include_directories(diag_gui_qt PRIVATE ${PROJECT_SOURCE_DIR}/dev/include)
    target_compile_options(diag_gui_qt PRIVATE -Wall -Wextra)
    target_link_libraries(diag_gui_qt PRIVATE Qt6::Core Qt6::Gui Qt6::Widgets Qt6::Network m)
    install(TARGETS diag_gui_qt RUNTIME DESTINATION bin)
    install(CODE [[
      execute_process(COMMAND /sbin/setcap cap_net_admin+ep "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/diag_gui_qt"
                      RESULT_VARIABLE SETCAP_RESULT
                      OUTPUT_VARIABLE SETCAP_STDOUT
                      ERROR_VARIABLE SETCAP_STDERR)
      if(SETCAP_RESULT EQUAL 0)
        message(STATUS "Granted cap_net_admin to diag_gui_qt")
      else()
        message(WARNING "Failed to grant cap_net_admin to diag_gui_qt: ${SETCAP_STDERR}${SETCAP_STDOUT}")
      endif()
    ]])
  else()
    message(STATUS "Qt6 not found: skipping diag_gui_qt target")
  endif()
endif()

# Motion demonstration script (runs against stc_server via icd_tx_tcp)
configure_file(dev/tests/turret_motion_demo.sh ${CMAKE_BINARY_DIR}/turret_motion_demo COPYONLY)
add_custom_target(turret_motion_demo_script ALL
  DEPENDS ${CMAKE_BINARY_DIR}/turret_motion_demo)


add_executable(test_x8_25_protocol
  dev/tests/test_x8_25_protocol.c
  dev/src/x8_25.c
)
target_include_directories(test_x8_25_protocol PRIVATE ${PROJECT_SOURCE_DIR}/dev/include)
target_compile_definitions(test_x8_25_protocol PRIVATE _DEFAULT_SOURCE)
target_compile_options(test_x8_25_protocol PRIVATE -Wall -Wextra)
target_link_libraries(test_x8_25_protocol PRIVATE m)

if(BUILD_TESTING)
  add_test(NAME test_actions COMMAND test_actions)
  add_test(NAME stc_state_test COMMAND stc_state_test)
  add_test(NAME stc_diag_test COMMAND stc_diag_test)
  add_test(NAME test_diag_codes COMMAND test_diag_codes)
  add_test(NAME test_diag_state_integration COMMAND test_diag_state_integration)
  add_test(NAME test_icd_codec COMMAND test_icd_codec)
  add_test(NAME test_myact_socketcan_ids COMMAND test_myact_socketcan_ids)
  add_test(NAME x8_25_test_can COMMAND x8_25_test_can)
  add_test(NAME test_x8_25_protocol COMMAND test_x8_25_protocol)
  add_test(NAME test_axis_cfg_map COMMAND test_axis_cfg_map)
  add_test(NAME stc_config_test COMMAND stc_config_test)
  add_test(NAME test_log_concurrency COMMAND test_log_concurrency)
  add_test(NAME test_can_worker_concurrency COMMAND test_can_worker_concurrency)
  add_test(NAME test_motion_backend_router COMMAND test_motion_backend_router)
endif()
