from SharsorIPCpp.PySharsorIPC import ServerFactory
from SharsorIPCpp.PySharsorIPC import StringTensorServer

from typing import Union

from SharsorIPCpp.PySharsor.extensions.ros_bridge.defs import NamingConventions

from SharsorIPCpp.PySharsorIPC import Journal, VLevel, LogType, dtype
from SharsorIPCpp.PySharsorIPC import toNumpyDType

import numpy as np

class FromRos():

    # Atomic bridge element to forward data from conventional Ros Topics 
    # on shared memory thorugh PySharsorIPCpp. Can be useful when developing
    # distributed architectures or when one needs to implement remote debugging features 

    # Given a basename and namespace, this object creates a suitable SharsorIPCpp server
    # which is created reading metadata on the conventional topics and updated with data 
    # streaming on topics in real time
    
    def __init__(self,
                basename: str,
                namespace: str = "",
                queue_size: int = 1, 
                ros_backend = "ros1",
                vlevel = VLevel.V3,
                verbose: bool = True,
                force_reconnection: bool = False):
        
        self._queue_size = queue_size

        self._basename = basename
        self._namespace = namespace

        self._vlevel = vlevel
        self._verbose = verbose
        self._force_reconnection = force_reconnection

        self._subscriber = None

        self._server = None

        self._ros_backend = ros_backend

        self._init_subscriber()

    def _check_backend(self):

        if not (self._ros_backend == "ros1" or \
                self._ros_backend == "ros1"):
            
            exception = f"Unsupported ROS backend. Supported are \"ros1\" and \"ros2\""

            Journal.log(self.__class__.__name__,
                        "_check_backend",
                        exception,
                        LogType.EXCEP,
                        throw_when_excep = True)
    
    def _init_subscriber(self):

        if self._ros_backend == "ros1":
            
            from SharsorIPCpp.PySharsor.extensions.ros_bridge.ros1_utils import Ros1Subscriber

            self._subscriber = Ros1Subscriber(basename = self._basename,
                                namespace = self._namespace,
                                queue_size = self._queue_size)
        
        elif self._ros_backend == "ros2":

            self._subscriber = ROS

            from SharsorIPCpp.PySharsor.extensions.ros_bridge.ros1_utils import Ros2Subscriber

            self._subscriber = Ros2Subscriber(basename = self._basename,
                                namespace = self._namespace,
                                queue_size = self._queue_size)

        else:
            
            exception = f"Backend {self._ros_backend} not supported. Please use either" + \
                    "\"ros1\" or \"ros2\"!"

            Journal.log(self.__class__.__name__,
                        "_init_subscriber",
                        exception,
                        LogType.EXCEP,
                        throw_when_excep = True)

    def _write_to_shared(self,
                    wait: bool = True):

        if wait:

            while not self._server.write(self._subscriber.preallocated_np_array[:, :], 0, 0):
                
                continue

            return True
        
        else:

            return self._server.write(self._subscriber.preallocated_np_array[:, :], 0, 0)
        
    def _synch_from_topic(self):
        
        self._subscriber.synch()
    
    def _to_sharsor_dtype(self,
                    np_dtype):

        if np_dtype == np.bool_:

            return dtype.Bool 
        
        if np_dtype == np.int32:
            
            return dtype.Int 
        
        if np_dtype == np.float32:
            
            return dtype.Float 
        
        if np_dtype == np.float64:

            return dtype.Double 
                
    def run(self):
        
        # first run subcriber

        self._subscriber.run() # blocking, waits for 
        # subscriber to initialize itself properly

        # creating a shared mem server
        self._server = ServerFactory(n_rows = self._subscriber.n_rows(), 
                    n_cols = self._subscriber.n_cols(),
                    basename = self._basename + "AAAAAA",
                    namespace = self._namespace, 
                    verbose = self._verbose, 
                    vlevel = self._vlevel, 
                    force_reconnection = self._force_reconnection, 
                    dtype = self._to_sharsor_dtype(self._subscriber.dtype()),
                    safe = True)
        
        self._server.run() # run server

    def stop(self):

        self._server.close()

    def update(self):
        
        self._subscriber.acquire_data() # blocking

        # success = self._write_to_shared() # updated publisher np view with shared memory
                
        return True