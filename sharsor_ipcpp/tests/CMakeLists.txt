find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

add_executable(server_test server_test.cpp)
target_link_libraries(server_test PRIVATE ${PROJECT_NAME} ${GTEST_BOTH_LIBRARIES})

#add_executable(client_test client_test.cpp)
#target_link_libraries(client_test PRIVATE ${PROJECT_NAME} ${GTEST_BOTH_LIBRARIES})

