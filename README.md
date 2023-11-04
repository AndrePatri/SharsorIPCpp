<h2 align="center"> <img src="https://img.shields.io/badge/License-GPLv2-purple.svg" alt="License"> SharsorIPCpp <img src="https://img.shields.io/badge/Docs-WIP-yellow" alt="Docs">

![icon.svg](docs/sphinx/source/_static/icon.svg)

</center></h2>

<!-- ![GitHub-Mark-Light](docs/icon-light.svg#gh-dark-mode-only)![GitHub-Mark-Dark](docs/icon-dark.svg#gh-light-mode-only) -->
Rt-friendly **shared tensors** built on top of **POSIX IPC** standards and [**Eigen**](https://eigen.tuxfamily.org/index.php?title=Main_Page) library, also shipped with Python bindings and [**NumPy**](https://numpy.org/) support.

At its core, SharsorIPCpp leverages *POSIX* *shared memory* and *semaphores* primitives in conjunction with Eigen's API to create shared tensors over **multiple processes**, which can then be safely accessed and manipulated in a **rt-compatible** way. 

SharsorIPCpp exposes to the user a convenient (multiple) `Client` - `Server` API to create, read, write and manage shared tensors from separate processes with **minimum latency**, and internally takes care of avoiding race conditions on the data.

At its current stage, SharsorIPCpp is templatized so as to support the creation of shared tensors of several datatypes (`bool`, `int`, `float` and `double`) and with different memory layouts (`column-major`, `row-major`). Additionally, SharsorIPCpp also exposes a `StringTensor` wrapper object designed for sharing arrays of UTF8 encoded-strings.

The library is also **fully binded** in Python, codename `PySharsorIPC`, and allows for seamless integration with the popular NumPy library.

For more details on what SharsorIPCpp offers, usage examples, performance benchmarks and so on and so forth, please have a look at the [documentation](https://andrepatri.github.io/SharsorIPCpp/v0.1.0/index.html) (WIP).

Continous integration status:
| *main* | *devel* |
|----------|----------|
| <img src="https://github.com/AndrePatri/SharsorIPCpp/actions/workflows/focal_CI_build_main.yml/badge.svg" alt="Focal CI">  | <img src="https://github.com/AndrePatri/SharsorIPCpp/actions/workflows/focal_CI_build_devel.yml/badge.svg" alt="CI Focal">  | 
| <img src="https://github.com/AndrePatri/SharsorIPCpp/actions/workflows/jammy_CI_build_main.yml/badge.svg" alt="CI Jammy">  | <img src="https://github.com/AndrePatri/SharsorIPCpp/actions/workflows/jammy_CI_build_devel.yml/badge.svg" alt="CI Jammy">  |

External dependencies: 
- [Eigen3](https://eigen.tuxfamily.org/index.php?title=Main_Page) - *required*: a C++ template library for linear algebra. On Linux, install it with ```sudo apt-get install libeigen3-dev```. Tensors on SharsorIPCpp are exposed, at the Cpp level, as either Eigen matrices or Eigen Maps of the underlying memory.
- [GoogleTest](https://github.com/google/googletest) - *optional*: a C++ testing framework. On Linux, install it with ```sudo apt-get install libgtest-dev```.
<!-- - **Real-time library** (rt) - *required*: ```sudo apt-get install librt-dev```
- **pthread** - *required*: the POSIX Threads library. On Linux, install it with ```sudo apt-get install libpthread-stubs0-dev``` -->

To compile the bindings (**PySharsorIPC**) you'll need: 
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
- Run-time semaphore acquisitions (used by `write` and `read`) are designed to be non-blocking and rt-safe. It is then user's responsibility to handle, if necessary, possible write/read failures due to semaphore acquisition.
- Calls to `run()/attach()` and `stop()` are not guaranteed to be rt-friendly. For rt applications, these calls should only be done during initialization/closing steps or, at run-time, sporadically.
- As of now, the logging utility `Journal` is not guaranteed to be rt-friendly. It is very useful for debugging purposes but, if working with rt-code, it is strongly recommended to set the verbosity level to `VLevel::V0` (which prints only exceptions) or to disable logging altogether with `verbose = false`.

ToDo:
- [ ] clean StringTensor bindings with and remove pythonic read and write methods
- [ ] make read and write methods of string tensor more efficient
- [ ] start to add documentation
  - [x] doc page initialization
  - [ ] fill with simple Cpp examples and also reference tests
  - [ ] Python interface examples
- [ ] generalize StringTensor to support actual 2D Tensors of strings (stack along columns)
- [ ] deploy on Anaconda 
