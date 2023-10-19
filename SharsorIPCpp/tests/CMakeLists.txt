find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS}
                    ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(read_write_bench read_write_bench.cpp)
target_link_libraries(read_write_bench PRIVATE ${PROJECT_NAME} ${GTEST_BOTH_LIBRARIES} pthread)

add_executable(server_conn_test server_conn_test.cpp)
target_link_libraries(server_conn_test PRIVATE ${PROJECT_NAME} ${GTEST_BOTH_LIBRARIES} pthread)

add_executable(client_conn_test client_conn_test.cpp)
target_link_libraries(client_conn_test PRIVATE ${PROJECT_NAME} ${GTEST_BOTH_LIBRARIES} pthread)

