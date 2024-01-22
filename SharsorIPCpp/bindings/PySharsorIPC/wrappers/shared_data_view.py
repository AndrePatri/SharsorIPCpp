import torch

import numpy as np

from SharsorIPCpp.PySharsorIPC import ServerFactory, ClientFactory
from SharsorIPCpp.PySharsorIPC import VLevel
from SharsorIPCpp.PySharsorIPC import RowMajor, ColMajor
from SharsorIPCpp.PySharsorIPC import toNumpyDType
from SharsorIPCpp.PySharsorIPC import dtype as sharsor_dtype 
from SharsorIPCpp.PySharsorIPC import Journal as Logger
from SharsorIPCpp.PySharsorIPC import LogType

from typing import List, Union
    
class SharedDataView:

    def __init__(self, 
            namespace = "",
            basename = "",
            is_server = False, 
            n_rows: int = -1, 
            n_cols: int = -1, 
            verbose: bool = False, 
            vlevel: VLevel = VLevel.V0,
            dtype: sharsor_dtype = sharsor_dtype.Float,
            fill_value = None,
            safe = True):

        self.basename = basename
        self.namespace = namespace

        self.fill_value = fill_value
        
        self.safe = safe

        self.verbose = verbose
        self.vlevel = vlevel

        self.is_server = is_server 

        self.shared_mem = None

        self.dtype = dtype

        self.layout = RowMajor
        if self.layout == RowMajor:

            self.order = 'C' # 'C'

        if self.layout == ColMajor:

            self.order = 'F' # 'F'

        if self.is_server:
            
            self.shared_mem = ServerFactory(n_rows = n_rows, 
                    n_cols = n_cols,
                    basename = self.basename,
                    namespace = self.namespace, 
                    verbose = self.verbose, 
                    vlevel = self.vlevel, 
                    force_reconnection = True, 
                    dtype = self.dtype,
                    layout = self.layout,
                    safe = self.safe)

        else:
            
            self.shared_mem = ClientFactory(
                    basename = self.basename,
                    namespace = self.namespace, 
                    verbose = self.verbose, 
                    vlevel = self.vlevel,
                    dtype = self.dtype,
                    safe = self.safe)
    
    def __del__(self):

        self.close()

    def _ensure2D(self, 
                data: Union[np.ndarray, 
                            torch.Tensor]):

        if isinstance(data, np.ndarray):

            if data.ndim != 2:

                return False
        
        elif isinstance(data, torch.Tensor):
            
            if data.ndim != 2:

                return False
            
        return True

    def _fits_into(self, 
                data1: Union[np.ndarray, 
                torch.Tensor], 
                data2: Union[np.ndarray, 
                torch.Tensor], 
                row_index: int, 
                col_index: int):

        # Check if array1 fits into array2 at specific indexes, 
        # assuming both are 2D arrays or tensors

        # Calculate ending indices
        end_row_index = row_index + data1.shape[0]
        end_col_index = col_index + data1.shape[1]

        # Check if data1 fits within the bounds of data2
        return end_row_index <= data2.shape[0] and end_col_index <= data2.shape[1]

    def get_n_clients(self):

        if self.is_server:

            return self.shared_mem.getNClients()
        
        else:

            message = "Number of clients can be retrieved only if is_server = True!!"

            Logger.log(self.__class__.__name__,
                "get_n_clients",
                message,
                LogType.EXCEP,
                throw_when_excep = False)
            
            return -1
            
    def run(self):
    
        if self.is_server:

            self.shared_mem.run()
        
        else:
            
            self.shared_mem.attach()

        self.n_rows = self.shared_mem.getNRows()
        self.n_cols = self.shared_mem.getNCols()

        # we create views as big as the underlying shared memory
        # in case only a portion of it is needed, this is not optimal
        # memory-wise. However, this way we gain in simplicity

        if self.fill_value is not None:
            
            self.numpy_view = np.full((self.n_rows, self.n_cols),
                            self.fill_value,
                            dtype=toNumpyDType(self.shared_mem.getScalarType()),
                            order=self.order
                            )

        else:
            
            self.numpy_view = np.zeros((self.n_rows, self.n_cols),
                                dtype=toNumpyDType(self.shared_mem.getScalarType()),
                                order=self.order 
                                )
        
        self.torch_view = torch.from_numpy(self.numpy_view) # changes in either the 
        # numpy or torch view will be reflected into the other one

        # also write fill value to shared memory

        if self.fill_value is not None and self.is_server:
                
                # view is initialized with NaN -> 
                # we write initialization
                self.synch_all(read = False, 
                        wait=True)
                
    def write(self, 
            data: Union[bool, int, float, 
                        np.float32, np.float64,
                        np.ndarray, 
                        torch.Tensor], 
            row_index: int, 
            col_index: int):
        
        if not self.shared_mem.isRunning:

            message = "You can only call write() if the run() method was previously called!"

            Logger.log(self.__class__.__name__,
                "write",
                message,
                LogType.EXCEP,
                throw_when_excep = False)
            
        if isinstance(data, (int, float, bool,
                            np.float32, np.float64)):  

            # write scalar into numpy view
            self.numpy_view[row_index, col_index] = data
            
            # write to shared memory
            return self.shared_mem.write(self.numpy_view[row_index:row_index + 1, 
                                                        col_index:col_index + 1], 
                                        row_index, col_index)

        if isinstance(data, torch.Tensor):  
            
            if not self._ensure2D(data):
                
                message = "Provided data should be 2D!!"

                Logger.log(self.__class__.__name__,
                    "write",
                    message,
                    LogType.EXCEP,
                    throw_when_excep = False)
                
                return False
            
            if not self._fits_into(data, self.torch_view, 
                                row_index, col_index):

                message = "Provided data does not fit in torch view!!"

                Logger.log(self.__class__.__name__,
                    "write",
                    message,
                    LogType.EXCEP,
                    throw_when_excep = False)

                return False
            
            input_rows, input_cols = data.shape

            # insert data into part of torch view
            self.torch_view[row_index:row_index + input_rows, 
                    col_index:col_index + input_cols] = data
            
            # write corresponding part of numpy view to memory
            return self.shared_mem.write(self.numpy_view[row_index:row_index + input_rows, 
                        col_index:col_index + input_cols], row_index, col_index)
        
        elif isinstance(data, np.ndarray):  # Check if data is a numpy array
            
            if not self._ensure2D(data):
                
                message = "Provided data should be 2D!!"

                Logger.log(self.__class__.__name__,
                    "write",
                    message,
                    LogType.EXCEP,
                    throw_when_excep = False)
                
                return False
            
            if not self._fits_into(data, self.torch_view, 
                                row_index, col_index):

                message = "Provided data does not fit in numpy view!!"

                Logger.log(self.__class__.__name__,
                    "write",
                    message,
                    LogType.EXCEP,
                    throw_when_excep = False)
                                
                return False
            
            input_rows, input_cols = data.shape

            # insert data into part of numpy view
            self.numpy_view[row_index:row_index + input_rows, 
                    col_index:col_index + input_cols] = data
            
            # write corresponding part of numpy view to memory
            return self.shared_mem.write(self.numpy_view[row_index:row_index + input_rows, 
                        col_index:col_index + input_cols], row_index, col_index)

        else:

            message = "Unsupported data type provided. " + \
                "Supported types are: bool, int, float, double, " + \
                "torch.Tensor, numpy.ndarray"

            Logger.log(self.__class__.__name__,
                "write",
                message,
                LogType.EXCEP,
                throw_when_excep = False)

            return False

    def write_wait(self,
            data: Union[bool, int, float, 
                        np.float32, np.float64,
                        np.ndarray, 
                        torch.Tensor], 
            row_index: int, 
            col_index: int):

        # tries writing until success

        while not self.write(data=data,
                    row_index=row_index,
                    col_index=col_index):
            
            continue

    def read(self, 
            row_index: int, 
            col_index: int, 
            data: Union[np.ndarray, 
                        torch.Tensor] = None):
        
        if not self.shared_mem.isRunning:

            message = "You can only call read() if the run() method was previously called!"

            Logger.log(self.__class__.__name__,
                "write",
                message,
                LogType.EXCEP,
                throw_when_excep = False)
            
        if isinstance(data, torch.Tensor):  
            
            if not self._ensure2D(data):
                
                message = "Provided data should be 2D!!"

                Logger.log(self.__class__.__name__,
                    "write",
                    message,
                    LogType.EXCEP,
                    throw_when_excep = False)
                
                return None, False
            
            if not self._fits_into(data, self.torch_view, 
                                row_index, col_index):

                message = "Provided data does not fit in torch view!!"

                Logger.log(self.__class__.__name__,
                    "write",
                    message,
                    LogType.EXCEP,
                    throw_when_excep = False)

                return None, False
            
            input_rows, input_cols = data.shape

            # update block of numpy view from shared memory
            success = self.shared_mem.read(self.numpy_view[row_index:row_index + input_rows, 
                    col_index:col_index + input_cols], row_index, col_index)
            
            if not success:

                return None, False
            
            # copy data into part of torch view
            data[:, :] = self.torch_view[row_index:row_index + input_rows, 
                    col_index:col_index + input_cols]
            
            return None, True
        
        elif isinstance(data, np.ndarray):  
            
            if not self._ensure2D(data):
                
                message = "Provided data should be 2D!!"

                Logger.log(self.__class__.__name__,
                    "write",
                    message,
                    LogType.EXCEP,
                    throw_when_excep = False)
                
                return None, False
            
            if not self._fits_into(data, self.torch_view, 
                                row_index, col_index):

                message = "Provided data does not fit in torch view!!"

                Logger.log(self.__class__.__name__,
                    "write",
                    message,
                    LogType.EXCEP,
                    throw_when_excep = False)

                return None, False
            
            input_rows, input_cols = data.shape

            # update block of numpy view from shared memory
            success = self.shared_mem.read(self.numpy_view[row_index:row_index + input_rows, 
                    col_index:col_index + input_cols], row_index, col_index)
            
            if not success:

                return None, False
            
            # copy data into part of numpy view
            data[:, :] = self.numpy_view[row_index:row_index + input_rows, 
                    col_index:col_index + input_cols]
            
            return None, True

        else:

            if data is None:

                # we return a scalar reading of the underlying shared memory
                success = self.shared_mem.read(self.numpy_view[row_index:row_index + 1, 
                    col_index:col_index + 1], row_index, col_index)
                
                return self.numpy_view[row_index:row_index + 1, 
                    col_index:col_index + 1], success
            
            else:

                # data is provided, but it's neither a ndarray nor a tensor

                message = "Provided data has to be either " + \
                "torch.Tensor, numpy.ndarray or None!"

                Logger.log(self.__class__.__name__,
                    "write",
                    message,
                    LogType.EXCEP,
                    throw_when_excep = False)

                return None, False

    def read_wait(self, 
            row_index: int, 
            col_index: int, 
            data: Union[np.ndarray, 
                        torch.Tensor] = None):
        
        # tries reading until success

        read_done = False

        while not read_done: 
            
            data_read, read_done = self.read(
                                row_index=row_index,
                                col_index=col_index,
                                data=data)
            
            if read_done:

                return data_read, read_done
            
            else:

                continue

    def synch(self, 
        row_index: int, 
        col_index: int, 
        n_rows: int,
        n_cols: int,
        read: bool = True):

        # synchs a block of the internal views from shared memory
        # or vice-versa
        
        # Calculate ending indices
        end_row_index = row_index + n_rows
        end_col_index = col_index + n_cols

        # Check it's possible to sin fits within the bounds 

        fits = (end_row_index <= self.numpy_view.shape[0] \
            and end_col_index <= self.numpy_view.shape[1])
        
        if not fits:

            return False
        
        if read:
            
            success = self.shared_mem.read(self.numpy_view[row_index:row_index + n_rows, 
                    col_index:col_index + n_cols], row_index, col_index)
            
            return success
            
        else:
            
            success = self.shared_mem.write(self.numpy_view[row_index:row_index + n_rows, 
                    col_index:col_index + n_cols], row_index, col_index)
            
            return success
    
    def synch_wait(self, 
        row_index: int, 
        col_index: int, 
        n_rows: int,
        n_cols: int,
        read: bool = True):

        while not self.synch(row_index=row_index,
                        col_index=col_index,
                        n_rows=n_rows,
                        n_cols=n_cols,
                        read=read):
            
            continue
        
    def synch_all(self, 
            read: bool = True, 
            wait = False):
        
        # synch whole view from shared memory

        if wait:
            
            self.synch_wait(row_index=0, col_index=0, 
                        n_rows=self.n_rows, n_cols=self.n_cols, 
                        read=read)
            
            return True

        else:

            return self.synch(row_index=0, col_index=0, 
                        n_rows=self.n_rows, n_cols=self.n_cols, 
                        read=read)

    def fill_with(self, 
            value: Union[bool, int, float, 
                        np.float32, np.float64]):

        # fill in place with unique value
        self.torch_view.fill_(value)

    def close(self):

        self.shared_mem.close()