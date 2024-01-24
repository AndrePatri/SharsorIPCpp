from abc import ABC, abstractmethod

from typing import List

from SharsorIPCpp.PySharsor.extensions.ros_bridge.defs import NamingConventions
from SharsorIPCpp.PySharsorIPC import Journal, VLevel, LogType

import numpy as np

class RosMessage(ABC):
    
    pass

class RosPublisher(ABC):

    def __init__(self,
                n_rows: int, 
                n_cols: int,
                basename: str,
                namespace: str = "",
                queue_size: int = 10,
                dtype = np.float32):
        
        self._topics = []
        
        self._basename = basename

        self._namespace = namespace 

        self._queue_size = queue_size

        self._dtype = dtype

        self._consistency_checks()

        self._naming_conv = NamingConventions()
        
        self._terminated = False

        self._ros_publishers = [None] * 4

        self.n_rows = n_rows
        self.n_cols = n_cols

        self.preallocated_ros_array = None

        self.preallocated_np_array = np.full(shape=(self.n_rows, self.n_cols), fill_value=np.nan, 
                                            dtype=self._dtype)

    def __del__(self):

        self.close()

    def _consistency_checks(self):
        
        if not isinstance(self._namespace, 
                    str):
            
            exception = f"namespace should be a string!"

            Journal.log(self.__class__.__name__,
                        "_consistency_checks",
                        exception,
                        LogType.EXCEP,
                        throw_when_excep = True)
        
        if not isinstance(self._basename, 
                    str):
            
            exception = f"basename should be a string!"

            Journal.log(self.__class__.__name__,
                        "_consistency_checks",
                        exception,
                        LogType.EXCEP,
                        throw_when_excep = True)
    
    def run(self):
        
        self._prerun()

        self._ros_publishers[0] = self._create_publisher(name=self._naming_conv.DataName(self._namespace, 
                                                                            self._basename),
                                                        dtype=self._dtype,
                                                        is_array=True,
                                                        queue_size=self._queue_size)
        
        self._ros_publishers[1] = self._create_publisher(self._naming_conv.nRowsName(self._namespace, 
                                                                            self._basename),
                                                        dtype=np.int32,
                                                        is_array=False,
                                                        queue_size=self._queue_size)
        
        self._ros_publishers[2] = self._create_publisher(self._naming_conv.nColsName(self._namespace, 
                                                                            self._basename),
                                                        dtype=np.int32,
                                                        is_array=False,
                                                        queue_size=self._queue_size)
        
        self._ros_publishers[3] = self._create_publisher(self._naming_conv.dTypeName(self._namespace, 
                                                                            self._basename),
                                                        dtype=np.int32,
                                                        is_array=False,
                                                        queue_size=self._queue_size)
        
    @abstractmethod
    def _prerun(self):

        pass

    @abstractmethod
    def _create_publisher(self,
                    name: str, 
                    namespace: str, 
                    dtype, 
                    queue_size: int):

        pass

    def close(self):

        if not self._terminated:
            
            self._close()

            self._terminated = True

    @abstractmethod
    def publish(self, 
            topic: str, 
            message: RosMessage):
        
        pass
    
    @abstractmethod
    def _close(self):
        
        pass

class RosSubscriber(ABC):

    def __init__(self,
                topics: List[str],
                rate: float):
        
        self._topics = topics
        
        self._rate = rate

        self._terminated = False

        self._init_subscriber()

    def __del__(self):

        self.shutdown()

    @abstractmethod
    def _init_subscriber(self):

        pass
    
    @abstractmethod
    def subscribe(self, 
            topic: str, 
            callback: RosMessage):
        
        pass
    
    @abstractmethod
    def shutdown(self):
        
        if not self._terminated:

            self._terminated = True
