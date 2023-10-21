find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS}
                    ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(read_write_bench read_write_bench.cpp)
target_link_libraries(read_write_bench PRIVATE ${PROJECT_NAME} ${GTEST_BOTH_LIBRARIES} pthread)


add_executable(server_conn_test server_conn_test.cpp)
target_link_libraries(server_conn_test PRIVATE ${PROJECT_NAME} ${GTEST_BOTH_LIBRARIES} pthread)

add_executable(client_conn_test client_conn_test.cpp)
target_link_libraries(client_conn_test PRIVATE ${PROJECT_NAME} ${GTEST_BOTH_LIBRARIES} pthread)

add_executable(jitter_test rt_jitter_test.cpp)
target_link_libraries(jitter_test PRIVATE ${PROJECT_NAME} ${GTEST_BOTH_LIBRARIES} pthread)

# Setting aux. variables
set(SERVER_CONN_TEST "server_conn_test")
set(CLIENT_CONN_TEST "client_conn_test")

# Configure the script using the paths of the test executables
configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/run_parallel_conn_tests.sh.in"
    "${CMAKE_CURRENT_BINARY_DIR}/run_parallel_conn_tests.sh"
    @ONLY
)

file(
    COPY "${CMAKE_CURRENT_BINARY_DIR}/run_parallel_conn_tests.sh"
    DESTINATION "${CMAKE_CURRENT_BINARY_DIR}"
    FILE_PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ
)

# Executable permissions
execute_process(
    COMMAND chmod +x "${CMAKE_CURRENT_BINARY_DIR}/run_parallel_conn_tests.sh"
)

gtest_discover_tests(read_write_bench)



