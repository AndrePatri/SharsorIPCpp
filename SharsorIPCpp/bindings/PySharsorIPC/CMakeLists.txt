cmake_minimum_required(VERSION 3.5)

add_compile_options(-std=c++17)

set(PyBindName "PySharsorIPC")

include_directories(${CMAKE_CURRENT_SOURCE_DIR} include/${PyBindName} src/)

find_package(pybind11 REQUIRED)

find_package(Eigen3 3.3 REQUIRED)

# backends (at least one of them needs to be found)
find_package(Torch QUIET)

#find_package(NumPy REQUIRED) # not needed-> already shipped with pybind

# setting up STATIC compilation against Torch
set(LIBTORCH_PATH "/usr/lib/Torch" CACHE PATH "Path to extracted pre-compiled Torch zip archive")
list(APPEND CMAKE_PREFIX_PATH "${LIBTORCH_PATH}")
set(TORCH_STATIC ON)
find_package(Torch REQUIRED STATIC)

message(STATUS "Compiling python bindings for ${LIBRARY_NAME}. Codename: ${PyBindName}")

# to get PYTHON_SITE
execute_process(
      COMMAND "${PYTHON_EXECUTABLE}" -c "if True:
        from distutils import sysconfig as sc
        print(sc.get_python_lib(prefix='', plat_specific=True))"
      OUTPUT_VARIABLE PYTHON_SITE
      OUTPUT_STRIP_TRAILING_WHITESPACE)

pybind11_add_module(${PyBindName} PySharsorIPC.cpp)
target_link_libraries(${PyBindName} PUBLIC SharsorIPCpp PRIVATE pybind11::module "${TORCH_LIBRARIES}")

install(TARGETS ${PyBindName}
    DESTINATION ${CMAKE_INSTALL_PREFIX}/${PYTHON_SITE}/${LIBRARY_NAME})

install(FILES __init__.py DESTINATION ${CMAKE_INSTALL_PREFIX}/${PYTHON_SITE}/${LIBRARY_NAME})

# install(TARGETS  DESTINATION "${PREFIX}/")
