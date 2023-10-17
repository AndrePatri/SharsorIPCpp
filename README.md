### <center> SharsorIPCpp </center>

![GitHub-Mark-Light](docs/icon-light.svg#gh-dark-mode-only)![GitHub-Mark-Dark](docs/icon-dark.svg#gh-light-mode-only)

Shared Tensors through Inter Process Communication (IPC) on C++ for POSIX-compatible OS.

External dependencies: 
- **Eigen3**: a C++ template library for linear algebra. On Linux, install it with ```sudo apt-get install libeigen3-dev```. Tensors on SharsorIPCpp are exposed, at the Cpp level, as either Eigen matrices or Eigen Maps of the underlying memory.
- **GTest** (Google Test): a C++ testing framework. On Linux, install it with ```sudo apt-get install libgtest-dev```.
- **Real-time library** (rt). ```sudo apt-get install librt-dev```
- **pthread**: the POSIX Threads library. On Linux, install it with ```sudo apt-get install libpthread-stubs0-dev```

The library is also shipped with Python bindings with both Numpy or PyTorch support. To be able to compile the bindings, you'll need the following packages:
- **pybind11**
- **Torch**
- **NumPy**
- **Eigen**

ToDo:
- [x] add ncols, nrows, dtype, nclients as shared data of type int 
- [x] add semaphore for data: when any kind of shared data has to be modified, this has to be acquired (either by the server or the client). The data semaphore will be held by the server, up to the point when the run() has finished. Also, when stopping the server, the semaphore is owned back y the server.
- [x] separate common utils between server and client
- [ ] write client: tries to acquire the data semaphore, read ncols, nrows,dtype, gets a memory view, increments the clients counter
- [ ] write python bindings: expose both a torch and numpy view of the tensors
- [ ] deploy on Anaconda   