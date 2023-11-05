cmake_minimum_required(VERSION 3.5)

add_compile_options(-std=c++17)

# Setting default visibility to hidden
set(CMAKE_CXX_VISIBILITY_PRESET hidden)
set(CMAKE_VISIBILITY_INLINES_HIDDEN 1)
# Helps in reducing the possibility of symbol clashes and also minimizes
# the chance of accidentally exporting symbols that aren't intended to be
# part of your library's public API

set(PyBindName "PySharsorIPC")
# set(BINDINGS_BINARY_DIR ${CMAKE_BINARY_DIR}/bindings/${LIBRARY_NAME}/${PyBindName})

include_directories(${CMAKE_CURRENT_SOURCE_DIR} include/ src/)

find_package(pybind11 REQUIRED)

find_package(Eigen3 3.3 REQUIRED)

message(STATUS "Compiling python bindings for ${LIBRARY_NAME}. Codename: ${PyBindName}")

# to get PYTHON_SITE
execute_process(
      COMMAND "${PYTHON_EXECUTABLE}" -c "if True:
        from distutils import sysconfig as sc
        print(sc.get_python_lib(prefix='', plat_specific=True))"
      OUTPUT_VARIABLE PYTHON_SITE
      OUTPUT_STRIP_TRAILING_WHITESPACE)

add_library("${PyBindName}Libs" STATIC src/PyStringTensor.cpp
                                       src/PyServer.cpp
                                       src/PyClient.cpp)
target_link_libraries("${PyBindName}Libs" PUBLIC SharsorIPCpp PRIVATE pybind11::module)
# set_target_properties("${PyBindName}Libs" PROPERTIES
#     LIBRARY_OUTPUT_DIRECTORY ${BINDINGS_BINARY_DIR}
#     RUNTIME_OUTPUT_DIRECTORY ${BINDINGS_BINARY_DIR})

pybind11_add_module(${PyBindName} PySharsorIPC.cpp)
target_link_libraries(${PyBindName} PRIVATE "${PyBindName}Libs" pybind11::module)
# set_target_properties(${PyBindName} PROPERTIES
#     LIBRARY_OUTPUT_DIRECTORY ${BINDINGS_BINARY_DIR}
#     RUNTIME_OUTPUT_DIRECTORY ${BINDINGS_BINARY_DIR})

install(TARGETS ${PyBindName}
    DESTINATION ${CMAKE_INSTALL_PREFIX}/${PYTHON_SITE}/${LIBRARY_NAME})

install(FILES __init__.py DESTINATION ${CMAKE_INSTALL_PREFIX}/${PYTHON_SITE}/${LIBRARY_NAME})

# install(TARGETS  DESTINATION "${PREFIX}/")
