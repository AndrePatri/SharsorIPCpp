add_executable(server_test server_test.cpp)
target_link_libraries(server_test PRIVATE SharsorIPC)

add_executable(client_test client_test.cpp)
target_link_libraries(client_test PRIVATE SharsorIPC)

