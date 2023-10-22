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

<!-- 
The library is also shipped with Python bindings with both Numpy or PyTorch support. To be able to compile the bindings, you'll need the following packages:
- **pybind11**
- **Torch**
- **NumPy**
- **Eigen** -->

If employed properly, the library is also designed to be rt-safe:
- Dynamic allocations are reduced to the bare minimum.
- Run-time semaphore acquisitions (used by `writeTensor` and `readTensor`) are designed to be non-blocking and rt-safe. It is then user's responsibility to handle, if necessary, possible write/read failures due to semaphore acquisition.
- Calls to `run()/attach()` and `stop()` are not guaranteed to be rt-friendly. For rt applications, these calls should only be done during initialization/closing steps or, at run-time, sporadically.
- As of now, the logging utility `Journal` is not guaranteed to be rt-friendly. It is very useful for debugging purposes but, if working with rt-code, it is strongly recommended to set the verbosity level to `VLevel::V0` (which prints only exceptions) or to disable logging altogether with `verbose = false`.

ToDo:
- [ ] python bindings with Numpy and PyTorch support
- [x] start to add documentation
- [ ] fill doc with simple examples
- [ ] generalize StringTensor to support actual 2D Tensors of strings (stack along columns)
- [ ] write tests for the Python bindings
- [ ] deploy on Anaconda 
- [x] continuous integration  
- [ ] test use from other package 
