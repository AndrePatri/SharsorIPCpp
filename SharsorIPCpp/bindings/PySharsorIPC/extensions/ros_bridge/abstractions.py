from abc import ABC, abstractmethod

from typing import List

from SharsorIPCpp.PySharsor.extensions.ros_bridge.defs import NamingConventions
from SharsorIPCpp.PySharsorIPC import Journal, VLevel, LogType

from perf_sleep.pyperfsleep import PerfSleep

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

        self._naming_conv = NamingConventions()

        self._dtype = dtype

        self._consistency_checks()

        self._ros_publishers = [None] * 4

        self._n_rows = n_rows
        self._n_cols = n_cols

        self.preallocated_ros_array = None

        self.preallocated_np_array = np.full(shape=(self._n_rows, self._n_cols), fill_value=np.nan, 
                                            dtype=self._dtype)
        
        self._terminated = False

    def __del__(self):

        self.close()

    def n_rows(self):

        return self._n_rows
    
    def n_cols(self):

        return self._n_cols
    
    def dtype(self):

        return self._dtype
    
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
               
    def _write_metadata(self):

        self._ros_publishers[1].publish(self._n_rows)

        self._ros_publishers[2].publish(self._n_cols)

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
                    dtype, 
                    queue_size: int,
                    is_array = False,
                    latch = True):

        pass

    def close(self):

        if not self._terminated:

            self._close()

            self._terminated = True
 
    @abstractmethod
    def _close(self):
        
        pass

class RosSubscriber(ABC):

    def __init__(self,
                basename: str,
                namespace: str = "",
                queue_size: int = 1):
        
        self._basename = basename

        self._namespace = namespace 

        self._queue_size = queue_size
        
        self._naming_conv = NamingConventions()

        self._ros_subscribers = [None] * 4
    
        # to be read from publisher
        self._n_rows = -1 
        self._n_cols = -1
        self._dtype = None

        self.preallocated_ros_array = None

        self.preallocated_np_array = None
        
        self._n_rows_retrieved = False
        self._n_cols_retrieved = False
        self._dtype_retrieved = False

        self._terminated = False

        self._init_sleep_time = 1e-5 # [s] 

        self._init_sleep_time_ns = int((self._init_sleep_time) * 1e+9)
        self._wait_sleep_time_ns = int((1e-6) * 1e+9)
    
        self._perf_timer = PerfSleep()

        self._writing_data = False

        self._reading_data = False

    def __del__(self):

        self.shutdown()

    def n_rows(self):

        return self._n_rows
    
    def n_cols(self):

        return self._n_cols
    
    def dtype(self):

        return self._dtype
    
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
                        "_decode_dtype",
                        exception,
                        LogType.EXCEP,
                        throw_when_excep = True)
            
    def _init_metadata_subs(self):
        
        self._ros_subscribers[1] = self._create_subscriber(name = self._naming_conv.nRowsName(
                                                                    self._namespace, 
                                                                    self._basename), 
                    dtype=np.int32,
                    callback=self._n_rows_callback, 
                    callback_args = None,
                    queue_size = self._queue_size,
                    is_array = False,
                    tcp_nodelay = False)

        self._ros_subscribers[2] = self._create_subscriber(name = self._naming_conv.nColsName(
                                                                    self._namespace, 
                                                                    self._basename), 
                    dtype=np.int32,
                    callback=self._n_cols_callback, 
                    callback_args = None,
                    queue_size = self._queue_size,
                    is_array = False,
                    tcp_nodelay = False)

        self._ros_subscribers[3] = self._create_subscriber(name = self._naming_conv.dTypeName(
                                                                    self._namespace, 
                                                                    self._basename), 
                    dtype=np.int32,
                    callback=self._dtype_callback, 
                    callback_args = None,
                    queue_size = self._queue_size,
                    is_array = False,
                    tcp_nodelay = False)
    
    def _init_data_subs(self):
        
        self._ros_subscribers[0] = self._create_subscriber(name = self._naming_conv.DataName(
                                                                    self._namespace, 
                                                                    self._basename), 
                    dtype=self._dtype,
                    callback=self._data_callback, 
                    callback_args = None,
                    queue_size = self._queue_size,
                    is_array = True,
                    tcp_nodelay = False)

    def _n_rows_callback(self,
                    msg):
        
        if not self._n_rows_retrieved:
            
            n_rows = int(msg.data)

            self._n_rows = n_rows

            self._n_rows_retrieved = True
        
        else:

            warning = f"New n_rows msg received on metadata topic."

            Journal.log(self.__class__.__name__,
                        "_n_rows_callback",
                        warning,
                        LogType.WARN,
                        throw_when_excep = True)
            
            n_rows = int(msg.data)

            if not self._n_rows == n_rows:

                # dimensions mismatch!!

                exception = f"New n_rows {n_rows} does not match previous {self._n_rows}"

                Journal.log(self.__class__.__name__,
                            "_n_rows_callback",
                            exception,
                            LogType.EXCEP,
                            throw_when_excep = True)

    def _n_cols_callback(self,
                    msg):

        if not self._n_cols_retrieved:
            
            n_cols = int(msg.data)

            self._n_cols = n_cols

            self._n_cols_retrieved = True
        
        else:

            warning = f"New n_cols msg received on metadata topic."

            Journal.log(self.__class__.__name__,
                        "_n_cols_callback",
                        warning,
                        LogType.WARN,
                        throw_when_excep = True)
            
            n_cols = int(msg.data)

            if not self._n_cols == n_cols:

                # dimensions mismatch!!

                exception = f"New n_rows {n_cols} does not match previous {self._n_cols}"

                Journal.log(self.__class__.__name__,
                            "_n_rows_callback",
                            exception,
                            LogType.EXCEP,
                            throw_when_excep = True)
                
    def _dtype_callback(self,
                    msg):

        if not self._dtype_retrieved:
            
            dtype = int(msg.data)

            self._dtype = self._decode_dtype(dtype)

            self._dtype_retrieved = True
        
        else:

            warning = f"New dtype msg received on metadata topic."

            Journal.log(self.__class__.__name__,
                        "_dtype_callback",
                        warning,
                        LogType.WARN,
                        throw_when_excep = True)
            
            dtype = int(msg.data)

            if not self._dtype == self._decode_dtype(dtype):

                # dimensions mismatch!!

                exception = f"New n_rows {dtype} does not match previous {self._dtype}"

                Journal.log(self.__class__.__name__,
                            "_dtype_callback",
                            exception,
                            LogType.EXCEP,
                            throw_when_excep = True)

    def acquire_data(self):

        while self._writing_data:

            self._perf_timer.clock_sleep(self._wait_sleep_time_ns)

    def _data_callback(self,
                    msg):
        
        self._writing_data = True

        # write data (also updated numpy view)
        self.preallocated_ros_array.data = msg.data

        # "release" data 
        self._writing_data = False
                
    def _got_metadata(self):
        
        metadata_retrieved = self._n_rows_retrieved and \
                self._n_cols_retrieved and \
                self._dtype_retrieved
        
        return metadata_retrieved

    def run(self):
        
        self._init_metadata_subs()

        while not self._got_metadata():
            
            # wait for metadata to be read on callbacks

            self._perf_timer.clock_sleep(self._init_sleep_time_ns)
        
        self._postrun()

        # self.preallocated_ros_array.data = np.full(shape=(self._n_rows, self._n_cols), 
        #                                     fill_value=np.nan, 
        #                                     dtype=self._dtype).flatten().tolist()
        
        # # view from buffer, this way changes reflect on both
        # self.preallocated_np_array[:, :] = np.frombuffer(self.preallocated_ros_array.data.data, 
        #                                             dtype=self._dtype).reshape((self._n_rows, 
        #                                                                        self._n_cols))
        
        self._init_data_subs()

        print("UIIIIIIIIIIIIIIIIIIII")

    def close(self):

        if not self._terminated:

            self._close()

            self._terminated = True

    @abstractmethod
    def _create_subscriber(self,
                    name: str, 
                    dtype, 
                    callback, 
                    callback_args = None,
                    queue_size: int = None,
                    is_array = False,
                    tcp_nodelay = False):

        pass

    @abstractmethod
    def _postrun(self):

        pass
    
    @abstractmethod
    def _close(self):
        
        pass
 
    