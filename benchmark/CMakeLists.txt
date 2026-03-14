# benchmark/CMakeLists.txt

add_executable(bench bench_main.cpp)

target_include_directories(bench PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}           # stats.h, latency_probe.h, report.h, can_sniffer.h
    ${CMAKE_SOURCE_DIR}/robstride_api/include
    ${CMAKE_SOURCE_DIR}/protocol
)

target_compile_options(bench PRIVATE -O2 -Wall -Wextra)

target_link_libraries(bench PRIVATE
    robstride_api
    Threads::Threads
    rt
)

set_target_properties(bench PROPERTIES OUTPUT_NAME "bench")