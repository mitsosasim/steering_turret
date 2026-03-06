cmake_minimum_required(VERSION 3.16)

project(STC C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(USE_SIMULATOR "Use software motor simulator" ON)

if(USE_SIMULATOR)
  add_library(simulator STATIC
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
  dev/src/pd4c.c
  dev/src/stc_actions.c
  dev/src/stc_limits.c
  dev/src/stc_state.c
  dev/src/stc_faults.c
  dev/src/stc_diag.c
)

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
  target_compile_definitions(stc_server PRIVATE SIMULATION)
  target_link_libraries(stc_server PRIVATE simulator)
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
  dev/src/stc_state.c
  dev/src/stc_faults.c
  dev/src/stc_diag.c
)

target_include_directories(stc_state_test PRIVATE ${PROJECT_SOURCE_DIR}/dev/include)
target_compile_options(stc_state_test PRIVATE -Wall -Wextra)

add_executable(stc_diag_test
  dev/tests/test_diag_codes.c
  dev/src/stc_diag.c
)

add_executable(test_icd_codec
  dev/tests/test_icd_codec.c
  dev/src/stc_icd.c
)

target_include_directories(test_icd_codec PRIVATE ${PROJECT_SOURCE_DIR}/dev/include)
target_compile_options(test_icd_codec PRIVATE -Wall -Wextra)

target_include_directories(stc_diag_test PRIVATE ${PROJECT_SOURCE_DIR}/dev/include)
target_compile_options(stc_diag_test PRIVATE -Wall -Wextra)


add_executable(stc_config_test
  dev/tests/stc_config_test.c
  dev/src/stc_config.c
  dev/src/stc_log.c
)

target_include_directories(stc_config_test PRIVATE ${PROJECT_SOURCE_DIR}/dev/include)
target_compile_options(stc_config_test PRIVATE -Wall -Wextra)
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
)

target_include_directories(test_can_worker_concurrency PRIVATE ${PROJECT_SOURCE_DIR}/dev/include)
target_compile_options(test_can_worker_concurrency PRIVATE -Wall -Wextra -Wno-unused-function)
target_link_libraries(test_can_worker_concurrency PRIVATE pthread m)

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
)

target_include_directories(test_can_worker_concurrency_tsan PRIVATE ${PROJECT_SOURCE_DIR}/dev/include)
target_compile_options(test_can_worker_concurrency_tsan PRIVATE -Wall -Wextra -Wno-unused-function -fsanitize=thread -fno-omit-frame-pointer)
target_link_options(test_can_worker_concurrency_tsan PRIVATE -fsanitize=thread -fno-omit-frame-pointer)
target_link_libraries(test_can_worker_concurrency_tsan PRIVATE pthread m)
