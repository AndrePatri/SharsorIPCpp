cmake_minimum_required(VERSION 3.5)
find_package(pybind11 REQUIRED)

find_package(Eigen3 3.3 REQUIRED)

# backends (at least one of them needs to be found)
find_package(Torch QUIET)

#find_package(NumPy REQUIRED)
set(LIBTORCH_PATH "/usr/lib/libtorch")

list(APPEND CMAKE_PREFIX_PATH "${LIBTORCH_PATH}")

set(TORCH_STATIC ON)
find_package(Torch REQUIRED STATIC)
#find_package(Torch REQUIRED)

#add_compile_options(-std=c++17)

message(STATUS "Compiling python bindings for ${LIBRARY_NAME}")

execute_process(
      COMMAND "${PYTHON_EXECUTABLE}" -c "if True:
        from distutils import sysconfig as sc
        print(sc.get_python_lib(prefix='', plat_specific=True))"
      OUTPUT_VARIABLE PYTHON_SITE
      OUTPUT_STRIP_TRAILING_WHITESPACE)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
pybind11_add_module(PySharsorIPC pysharsor_ipc.cpp)
target_link_libraries(PySharsorIPC PUBLIC SharsorIPCpp PRIVATE pybind11::module "${TORCH_LIBRARIES}")

install(TARGETS PySharsorIPC
            DESTINATION ${CMAKE_INSTALL_PREFIX}/${PYTHON_SITE}/${LIBRARY_NAME})

install(FILES __init__.py DESTINATION ${CMAKE_INSTALL_PREFIX}/${PYTHON_SITE}/${LIBRARY_NAME})

# install(TARGETS  DESTINATION "${PREFIX}/")
