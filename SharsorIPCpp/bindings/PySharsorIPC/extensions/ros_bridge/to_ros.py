from SharsorIPCpp.PySharsorIPC import Client
from SharsorIPCpp.PySharsorIPC import StringTensorClient

from typing import Union

from SharsorIPCpp.PySharsor.extensions.ros_bridge.defs import NamingConventions

from SharsorIPCpp.PySharsorIPC import Journal, VLevel, LogType
from SharsorIPCpp.PySharsorIPC import toNumpyDType
from SharsorIPCpp.PySharsor.extensions.ros_bridge.ros1_utils import Ros1Publisher

class ToRos():

    # Atomic bridge element to forward data from SharsorIPCpp or PySharsorIPCpp
    # over topic. Can be useful when developing distributed architectures or when 
    # one needs to implement remote debugging features 

    # Given a client object (either a normal client or a string client), 
    # this object creates a publisher to a number of conventional topics to
    # stream both data and meta data associated with Sharsor.

    def __init__(self,
                client: Union[Client, 
                            StringTensorClient],
                queue_size: int = 1, # by default only read latest msg
                ros_backend = "ros1"):

        self._check_client(client)

        self._client = client

        self._queue_size = queue_size

        self._publisher = None

        self._ros_backend = ros_backend

    def _check_client(self,
                client: Union[Client, 
                            StringTensorClient]):

        if not isinstance(client, 
                    (Client, StringTensorClient)):

            exception = f"Provided client should be either an instance of ClientFactory" + \
                " or of StringTensorClient classes!"

            Journal.log(self.__class__.__name__,
                        "_check_client",
                        exception,
                        LogType.EXCEP,
                        throw_when_excep = True)
    
    def _check_backend(self):

        if not (self._ros_backend == "ros1" or \
                self._ros_backend == "ros1"):
            
            exception = f"Unsupported ROS backend. Supported are \"ros1\" and \"ros2\""

            Journal.log(self.__class__.__name__,
                        "_check_client",
                        exception,
                        LogType.EXCEP,
                        throw_when_excep = True)
    
    def _synch_from_shared_mem(self,
                    wait: bool = True):

        if wait:

            while not self._client.read(self._publisher.preallocated_np_array[:, :], 0, 0):
                
                continue

            return True
        
        else:

            return self._client.read(self._publisher.preallocated_np_array[:, :], 0, 0)
        
    def run(self,
            latch: bool = True):

        if not self._client.isRunning():
                        
            # manually run client if not running

            self._client.attach()
        
        if self._ros_backend == "ros1":

            self._publisher = Ros1Publisher(n_rows = self._client.getNRows(), 
                        n_cols = self._client.getNCols(),
                        basename = self._client.getBasename(),
                        namespace = self._client.getNamespace(),
                        queue_size = self._queue_size,
                        dtype = toNumpyDType(self._client.getScalarType()))
        
        elif self._ros_backend == "ros2":

            exception = f"ros2 backend not yet supported!"

            Journal.log(self.__class__.__name__,
                        "_check_client",
                        exception,
                        LogType.EXCEP,
                        throw_when_excep = True)

        self._publisher.run(latch=latch) # initialized topics and writes initializations
    
    def stop(self):

        self._client.close()

    def update(self):
        
        success = self._synch_from_shared_mem() # updated publisher np view with shared memory

        self._publisher.pub_data()
                
        return success