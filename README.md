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
- [x] write client: tries to acquire the data semaphore, read ncols, nrows,dtype, gets a memory view, increments the clients counter
- [ ] add method to read a block and not all the Tensor
- [ ] write a "PyFace" interface on top of SharspIPCpp which is a simple wrapper around its public methods ("PyServer", "PyClient"), but it exposes a write method which accepts either a Torch tensor or a Numpy 2D array and a position where this has to be written and a read method which returns a torch or numpy block of the data. No view of the data should be exposed so that race conditions will not be possible at the Python level.
- [ ] add a class for shared string lists built on top of the server and clients. Ideally, we would want it to be templatized, so that a server type is passed, then a server is created, otherwise a client.
- [ ] write tests for the Python bindings
- [ ] deploy on Anaconda   
- [ ] test use from other package 
- [ ] deploy to the world (make public)