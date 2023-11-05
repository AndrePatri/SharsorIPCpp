find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS}
                    ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(read_write_bench read_write_bench.cpp)
target_link_libraries(read_write_bench PRIVATE ${PROJECT_NAME} ${GTEST_BOTH_LIBRARIES} pthread)

add_executable(consistency_checks_clnt consistency_checks_clnt.cpp)
target_link_libraries(consistency_checks_clnt PRIVATE ${PROJECT_NAME} ${GTEST_BOTH_LIBRARIES} pthread)

add_executable(consistency_checks_srvr consistency_checks_srvr.cpp)
target_link_libraries(consistency_checks_srvr PRIVATE ${PROJECT_NAME} ${GTEST_BOTH_LIBRARIES} pthread)

add_executable(server_conn_test server_conn_test.cpp)
target_link_libraries(server_conn_test PRIVATE ${PROJECT_NAME} ${GTEST_BOTH_LIBRARIES} pthread)

#add_executable(client_conn_test client_conn_test.cpp)
#target_link_libraries(client_conn_test PRIVATE ${PROJECT_NAME} ${GTEST_BOTH_LIBRARIES} pthread)

#add_executable(jitter_test rt_jitter_test.cpp)
#target_link_libraries(jitter_test PRIVATE ${PROJECT_NAME} ${GTEST_BOTH_LIBRARIES} pthread)

# Setting aux. variables
set(CONSISTENCY_CHECKS_CLIENT "consistency_checks_clnt")
set(CONSISTENCY_CHECKS_SERVER "consistency_checks_srvr")

# Configure the script using the paths of the test executables
configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/run_consistency_check.sh.in"
    "${CMAKE_CURRENT_BINARY_DIR}/run_consistency_check.sh"
    @ONLY
)

file(
    COPY "${CMAKE_CURRENT_BINARY_DIR}/run_consistency_check.sh"
    DESTINATION "${CMAKE_CURRENT_BINARY_DIR}"
    FILE_PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ
)
# Executable permissions
execute_process(
    COMMAND chmod +x "${CMAKE_CURRENT_BINARY_DIR}/run_consistency_check.sh"
)

# copying pybinds tests to build folder
add_custom_command(
    OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/PySharsorIPC"
    COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${CMAKE_CURRENT_SOURCE_DIR}/PySharsorIPC"
            "${CMAKE_CURRENT_BINARY_DIR}/PySharsorIPC"
    COMMENT "Copying PySharsorIPC python unittests directory to build folder"
)
# Create a custom target that depends on the copied directory
add_custom_target(copy_pysharsoripc_tests ALL
    DEPENDS "${CMAKE_CURRENT_BINARY_DIR}/PySharsorIPC"
)

# install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/PySharsorIPC/
#     DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/PySharsorIPC
#     FILES_MATCHING PATTERN "*.py")

gtest_discover_tests(read_write_bench)
#gtest_discover_tests(consistency_checks_srvr)
#gtest_discover_tests(consistency_checks_clnt)

