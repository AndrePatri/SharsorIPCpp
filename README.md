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

As a side note, the library is also designed to be rt-safe:
- No dynamic allocations are performed
- Run-time semaphore acquisitions (used by `writeTensor(const Tensor<Scalar>& data,
                            int row = 0,
                            int col = 0) ` and `readTensor(Tensor<Scalar>& output,
                            int row = 0, int col = 0)`) are designed to be non-blocking. It is then user's responsibility to preallocate, e.g. the output tensor and handle write/read failures due to semaphore acquisition.
- Calls to `run()/attach()` and `stop()` and non-rt safe by default and should be done during initialization/closing phases or, at run-time, sporadically.
- As of now, the logging utility `Journal` is not rt-safe. So, if working with rt-code, it is strongly recommended to set the verbosity level to `VLevel::V0` (which allows only exceptions) or disable logging altogether with `verbose = false`.
ToDo:
- [x] add ncols, nrows, dtype, nclients as shared data of type int 
- [x] add semaphore for data: when any kind of shared data has to be modified, this has to be acquired (either by the server or the client). The data semaphore will be held by the server, up to the point when the run() has finished. Also, when stopping the server, the semaphore is owned back y the server.
- [x] separate common utils between server and client
- [x] write client: tries to acquire the data semaphore, read ncols, nrows,dtype, gets a memory view, increments the clients counter
- [x] add method to read a block and not all the Tensor
- [ ] modify Journal to be rt safe by using fixed size arrays and char*.
- [ ] add a class for shared string lists built on top of the server and clients. Ideally, we would want it to be templatized, so that a server type is passed, then a server is created, otherwise a client.
- [ ] write a "PyFace" interface on top of SharspIPCpp which is a simple wrapper around its public methods ("PyServer", "PyClient"), but it exposes a write method which accepts either a Torch tensor or a Numpy 2D array and a position where this has to be written and a read method which returns a torch or numpy block of the data. No view of the data should be exposed so that race conditions will not be possible at the Python level.
- [ ] write tests for the Python bindings
- [ ] move read/write methods to non-blocking sem_trywait to allow the use in rt loops. Basically, this means that when called, writeTensor and readTensor might also fail due to the failed semaphore acquisition. The run()/attach() and stop() methods can instead be blocking because they are thought to be used in initialization phases or sporadically at runtime.
- [ ] deploy on Anaconda   
- [ ] test use from other package 
- [ ] deploy to the world (make public)