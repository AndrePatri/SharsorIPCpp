find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS}
                    ${CMAKE_CURRENT_SOURCE_DIR}/include)

message(STATUS "###")

add_executable(read_write_bench read_write_bench.cpp)
target_link_libraries(read_write_bench PRIVATE ${PROJECT_NAME} ${GTEST_BOTH_LIBRARIES} pthread)

add_executable(connection_test1 connection_test1.cpp)
target_link_libraries(connection_test1 PRIVATE ${PROJECT_NAME} ${GTEST_BOTH_LIBRARIES} pthread)

#add_executable(connection_test2 connection_test2.cpp)
#target_link_libraries(connection_test2 PRIVATE ${PROJECT_NAME} ${GTEST_BOTH_LIBRARIES} pthread)

