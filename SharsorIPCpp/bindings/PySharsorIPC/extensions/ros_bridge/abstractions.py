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
                queue_size: int = 1, # by default only read latest msg
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
        
    def _encode_dtype(self, numpy_dtype):
        
        if numpy_dtype == np.bool_:

            return 0
        
        elif numpy_dtype == np.int32:

            return 1
        
        elif numpy_dtype == np.float32:

            return 2
        
        elif numpy_dtype == np.float64:

            return 3
        
        else:
            
            exception = f"Unsupported NumPy data type: {numpy_dtype}"

            Journal.log(self.__class__.__name__,
                        "_encode_dtype",
                        exception,
                        LogType.EXCEP,
                        throw_when_excep = True)
                    
    def _decode_dtype(self, dtype_code: int):

        if dtype_code == 0:

            return np.bool_
        
        elif dtype_code == 1:

            return np.int32
        
        elif dtype_code == 2:
            
            return np.float32
        
        elif dtype_code == 3:

            return np.float64
        
        else:
            
            exception = f"Unsupported encoded integer: {dtype_code}"

            Journal.log(self.__class__.__name__,
                        "_encode_dtype",
                        exception,
                        LogType.EXCEP,
                        throw_when_excep = True)
    
    def _write_metadata(self):

        self._ros_publishers[1].publish(self.n_rows)

        self._ros_publishers[2].publish(self.n_cols)

        self._ros_publishers[3].publish(self._encode_dtype(self._dtype))
        
    def _write_data_init(self):

        self._ros_publishers[0].publish(self.preallocated_ros_array)

    def pub_data(self):

        # writes latest value in preallocated_np_array

        self.preallocated_ros_array.data = self.preallocated_np_array.flatten().tolist()
        
        self._ros_publishers[0].publish(self.preallocated_ros_array)

    def run(self,
            latch: bool = True):
        
        self._prerun()

        self._ros_publishers[0] = self._create_publisher(name=self._naming_conv.DataName(self._namespace, 
                                                                            self._basename),
                                                        dtype=self._dtype,
                                                        is_array=True,
                                                        queue_size=self._queue_size,
                                                        latch=latch)
        
        self._ros_publishers[1] = self._create_publisher(self._naming_conv.nRowsName(self._namespace, 
                                                                            self._basename),
                                                        dtype=np.int32,
                                                        is_array=False,
                                                        queue_size=self._queue_size,
                                                        latch=latch)
        
        self._ros_publishers[2] = self._create_publisher(self._naming_conv.nColsName(self._namespace, 
                                                                            self._basename),
                                                        dtype=np.int32,
                                                        is_array=False,
                                                        queue_size=self._queue_size,
                                                        latch=latch)
        
        self._ros_publishers[3] = self._create_publisher(self._naming_conv.dTypeName(self._namespace, 
                                                                            self._basename),
                                                        dtype=np.int32,
                                                        is_array=False,
                                                        queue_size=self._queue_size,
                                                        latch=latch)
        
        self._write_metadata()
        self._write_data_init()
        
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
