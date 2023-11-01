### <center> SharsorIPCpp </center>

<!-- ![GitHub-Mark-Light](docs/icon-light.svg#gh-dark-mode-only)![GitHub-Mark-Dark](docs/icon-dark.svg#gh-light-mode-only) -->
![icon.svg](docs/sphinx/source/_static/icon.svg)
Rt-safe Shared Tensors through Inter Process Communication (IPC) on C++ for POSIX-compatible OS.

<center>
<table>
  <tc>
    <td><img src="https://img.shields.io/badge/License-GPLv2-purple.svg" alt="License"></td>
    <td><img src="https://img.shields.io/badge/Docs-WIP-yellow" alt="Docs"></td>
  </tc>
  <td colspan="2"> <!-- This will span the cell across two columns to accommodate the nested table -->
  <table>
    <tr>
      <td><img src="https://github.com/AndrePatri/SharsorIPCpp/actions/workflows/focal_CI_build_devel.yml/badge.svg" alt="CI Focal"></td>
    </tr>
    <tr>
      <td><img src="https://github.com/AndrePatri/SharsorIPCpp/actions/workflows/jammy_CI_build_devel.yml/badge.svg" alt="CI Jammy"></td>
    </tr>
  </table>
  </td>
</table>
</center>

The documentation (WIP currently) is host [here](https://andrepatri.github.io/SharsorIPCpp/v0.1.0/index.html).

External dependencies: 
- [Eigen3](https://eigen.tuxfamily.org/index.php?title=Main_Page) - *required*: a C++ template library for linear algebra. On Linux, install it with ```sudo apt-get install libeigen3-dev```. Tensors on SharsorIPCpp are exposed, at the Cpp level, as either Eigen matrices or Eigen Maps of the underlying memory.
- [GoogleTest](https://github.com/google/googletest) - *optional*: a C++ testing framework. On Linux, install it with ```sudo apt-get install libgtest-dev```.
<!-- - **Real-time library** (rt) - *required*: ```sudo apt-get install librt-dev```
- **pthread** - *required*: the POSIX Threads library. On Linux, install it with ```sudo apt-get install libpthread-stubs0-dev``` -->

To compile python bindings you'll need: 
- [Eigen3](https://eigen.tuxfamily.org/index.php?title=Main_Page) - *required*
- [pybind11](https://github.com/pybind/pybind11) - *required*. 

<!-- Run-time dependencies for the bindings:
- **linux-vdso**
- **librt**
- **libstdc++**
- **libgcc**
- **libc**
- **libpthread**
- **libm** -->

If employed properly, the library is also designed to be rt-safe (in C++):
- Dynamic allocations are reduced to the bare minimum.
- Run-time semaphore acquisitions (used by `writeTensor` and `readTensor`) are designed to be non-blocking and rt-safe. It is then user's responsibility to handle, if necessary, possible write/read failures due to semaphore acquisition.
- Calls to `run()/attach()` and `stop()` are not guaranteed to be rt-friendly. For rt applications, these calls should only be done during initialization/closing steps or, at run-time, sporadically.
- As of now, the logging utility `Journal` is not guaranteed to be rt-friendly. It is very useful for debugging purposes but, if working with rt-code, it is strongly recommended to set the verbosity level to `VLevel::V0` (which prints only exceptions) or to disable logging altogether with `verbose = false`.

Usage notes:
- The library is templated and this allows the user to choose the dtype to be employed for the elements of the tensor (`bool`, `int`, `float` and `double` are supported) and also on the memory layout (defaults to `ColMajor`, but `RowMajor` is also supported).
- Additionally, a `StringTensor` wrapper on top of the `Server` and `Client` allows to create and manage UTF8-encoded tensors of strings. 
- Python bindings for the `Server`, `Client` and `StringTensor` are provided. They are built on top of `PyServer` and `PyClient` C++ wrappers classes and provide and, thanks to **pybind11**, they provide seamless integration with [NumPy](https://numpy.org/).
- For usage examples, please have a look at the test folder or at the [documentation](https://andrepatri.github.io/SharsorIPCpp/v0.1.0/index.html) (WIP).

ToDo:
- [x] benchmark dynamic allocations in PyClient and PyServer. Is it really necessary to implement numpy-tailored child classes of PyClient and PyServer?
- [x] Benchmark Client/ServerWrapper approach wrt bare bindings. How much perf overhead?
- [x] bind PyClient and PyServer
- [ ] clean StringTensor bindings with and remove pythonic read and write methods
- [ ] add python unit tests for the bindings (performance and consistency checks)
- [ ] make read and write methods of string tensor more efficient (multithreaded encoding?)
- [x] make all gtests types (dtype and layout)
- [x] add roundtrip consistency checks to CI
- [] start to add documentation
  - [x] doc page initialization
  - [ ] fill with simple Cpp examples and also reference tests
  - [ ] Python interface examples
- [ ] generalize StringTensor to support actual 2D Tensors of strings (stack along columns)
- [ ] test use from other package (check linking) 
- [ ] deploy on Anaconda 
