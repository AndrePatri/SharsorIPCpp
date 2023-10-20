cmake_minimum_required(VERSION 3.5)
find_package(pybind11 REQUIRED)

find_package(Eigen3 3.3 REQUIRED)

# backends (at least one of them needs to be found)
find_package(Torch QUIET)

find_package(NumPy QUIET)

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
target_link_libraries(PySharsorIPC PUBLIC SharsorIPCpp PRIVATE pybind11::module)

install(TARGETS PySharsorIPC
            DESTINATION ${CMAKE_INSTALL_PREFIX}/${PYTHON_SITE}/${LIBRARY_NAME})

install(FILES __init__.py DESTINATION ${CMAKE_INSTALL_PREFIX}/${PYTHON_SITE}/${LIBRARY_NAME})

# install(TARGETS  DESTINATION "${PREFIX}/")
