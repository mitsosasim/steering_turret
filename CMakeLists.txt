cmake_minimum_required(VERSION 3.16)

project(STC C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

add_executable(stc_server
  dev/src/stc_server.c
  dev/src/stc_icd.c
  dev/src/pd4c.c
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
target_include_directories(icd_tx_tcp PRIVATE ${PROJECT_SOURCE_DIR}/dev/include)
target_include_directories(tlm_rx_udp PRIVATE ${PROJECT_SOURCE_DIR}/dev/include)

target_compile_options(stc_server PRIVATE -Wall -Wextra)
target_compile_options(icd_tx_tcp PRIVATE -Wall -Wextra)
target_compile_options(tlm_rx_udp PRIVATE -Wall -Wextra)

target_link_libraries(stc_server PRIVATE pthread m)
