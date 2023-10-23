### <center> SharsorIPCpp </center>

<!-- ![GitHub-Mark-Light](docs/icon-light.svg#gh-dark-mode-only)![GitHub-Mark-Dark](docs/icon-dark.svg#gh-light-mode-only) -->
![icon.svg](docs/sphinx/source/_static/icon.svg)
Rt-safe Shared Tensors through Inter Process Communication (IPC) on C++ for POSIX-compatible OS.

<center>
<table>
  <tr>
    <td><img src="https://img.shields.io/badge/License-GPLv2-purple.svg" alt="License"></td>
    <td><img src="https://img.shields.io/badge/Docs-WIP-yellow" alt="Docs"></td>
    <td><img src="https://app.travis-ci.com/AndrePatri/SharsorIPCpp.svg?branch=main" alt="CI"></td>
  </tr>
</table>
</center>

The documentation (WIP currently) is host [here](https://andrepatri.github.io/SharsorIPCpp/v0.1.0/index.html).

External dependencies: 
- **Eigen3** - *required*: a C++ template library for linear algebra. On Linux, install it with ```sudo apt-get install libeigen3-dev```. Tensors on SharsorIPCpp are exposed, at the Cpp level, as either Eigen matrices or Eigen Maps of the underlying memory.
- **GTest** (Google Test) - *optional*: a C++ testing framework. On Linux, install it with ```sudo apt-get install libgtest-dev```.
- **Real-time library** (rt) - *required*: ```sudo apt-get install librt-dev```
- **pthread** - *required*: the POSIX Threads library. On Linux, install it with ```sudo apt-get install libpthread-stubs0-dev```

Python bindings dependencies: 
- **pybind11** - *required*. Already shipped with Numpy support.
- **Eigen3** - *required*
- **Torch** - *required*. The bindings link against libtorch. As of now, the only alternative is to download a pre-compiled zip archive from [here](https://download.pytorch.org/libtorch/cpu/libtorch-shared-with-deps-2.1.0%2Bcpu.zip). Unzip the archive to whatever location you like, then turn on the WITH_PYTHON flag and set the LIBTORCH_PATH to the root of the directory of the extracted zip archive. Internally this is also added to the RPATH associated with the bindings through CMake.

<!-- 
The library is also shipped with Python bindings with both Numpy or PyTorch support. To be able to compile the bindings, you'll need the following packages:
- **pybind11**
- **Torch**
- **NumPy**
- **Eigen** -->

If employed properly, the library is also designed to be rt-safe (in C++):
- Dynamic allocations are reduced to the bare minimum.
- Run-time semaphore acquisitions (used by `writeTensor` and `readTensor`) are designed to be non-blocking and rt-safe. It is then user's responsibility to handle, if necessary, possible write/read failures due to semaphore acquisition.
- Calls to `run()/attach()` and `stop()` are not guaranteed to be rt-friendly. For rt applications, these calls should only be done during initialization/closing steps or, at run-time, sporadically.
- As of now, the logging utility `Journal` is not guaranteed to be rt-friendly. It is very useful for debugging purposes but, if working with rt-code, it is strongly recommended to set the verbosity level to `VLevel::V0` (which prints only exceptions) or to disable logging altogether with `verbose = false`.

ToDo:
- [x] continuous integration  
- [ ] python bindings with Numpy and PyTorch support
  - [x] bind StringTensor
  - [ ] bind Client and Server: write factory method, so that it's not necessary to have a different name for each different type of data
  - [ ] add support for Torch and Eigen
  - [ ] write some read/write unittests to benchmark performance impacts of additional dynamic allocations in bindings
  - [ ] add python tests to CI
- [] start to add documentation
  - [x] doc page initialization
  - [ ] fill with simple Cpp examples and also reference tests
  - [ ] Python interface examples
- [ ] generalize StringTensor to support actual 2D Tensors of strings (stack along columns)
- [ ] deploy on Anaconda 
- [ ] test use from other package 
