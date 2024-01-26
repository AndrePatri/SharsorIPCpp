from SharsorIPCpp.PySharsor.extensions.ros_bridge.abstractions import RosPublisher
from SharsorIPCpp.PySharsor.extensions.ros_bridge.abstractions import RosSubscriber

import rospy

from std_msgs.msg import Bool, Int32, Float32, Float64
from std_msgs.msg import Int32MultiArray, Float32MultiArray, Float64MultiArray
from rospy.numpy_msg import numpy_msg

from std_msgs.msg import String

import numpy as np

def toRosDType(numpy_dtype,
            is_array: False):

        if numpy_dtype == np.bool_:

            return Int32MultiArray if is_array else Bool
        
        elif numpy_dtype == np.int32:
            
            return Int32MultiArray if is_array else Int32
        
        elif numpy_dtype == np.float32:

            return Float32MultiArray if is_array else Float32
        
        elif numpy_dtype == np.float64:

            return Float64MultiArray if is_array else Float64
        
        else:

            raise ValueError(f"Unsupported NumPy data type: {numpy_dtype}")
        
class Ros1Publisher(RosPublisher):

    def __init__(self,
            n_rows: int, 
            n_cols: int,
            basename: str,
            namespace: str = "",
            queue_size: int = 1, # by default only read latest msg
            dtype = np.float32,
            latch: bool = True):

        self._latch = latch

        super().__init__(n_rows=n_rows, 
                n_cols=n_cols,
                basename=basename,
                namespace=namespace,
                queue_size=queue_size, # by default only read latest msg
                dtype=dtype)

        self._use_np_directly = True

        self._to_list_first = True # override default

    def _create_publisher(self,
                    name: str, 
                    dtype, 
                    queue_size: int,
                    is_array = False):

        # use numpy msg for efficiency
        publisher = rospy.Publisher(name =name, 
                        data_class=numpy_msg(toRosDType(dtype, is_array)), 
                        queue_size=queue_size,
                        latch=self._latch)
        
        return publisher
    
    def _prerun(self):

        pass

        # pre-allocate stuff
        # self.preallocated_ros_array = toRosDType(numpy_dtype=self._dtype,
        #                             is_array=True)()

        # self.preallocated_ros_array.data = self.preallocated_np_array.flatten().tolist()

    def _close(self):
        
        # called in the close()

        for i in range(len(self._ros_publishers)):

                if self._ros_publishers[i] is not None:

                    self._ros_publishers[i].unregister()
            
class Ros1Subscriber(RosSubscriber):

    def __init__(self,
                basename: str,
                namespace: str = "",
                queue_size: int = 1,
                tcp_nodelay = False):

        self._tcp_nodelay = tcp_nodelay
        
        super().__init__(basename=basename,
                namespace=namespace,
                queue_size=queue_size)

        self._use_np_directly = True

    def _create_subscriber(self,
                    name: str, 
                    dtype, 
                    callback, 
                    queue_size: int = None,
                    is_array = False):

        # use numpy msg for efficiency
        subscriber = rospy.Subscriber(name =name, 
                        data_class=numpy_msg(toRosDType(dtype, is_array)), 
                        callback=callback,
                        queue_size=queue_size, 
                        tcp_nodelay=self._tcp_nodelay
                        )
        
        return subscriber
    
    def _postrun(self):
        
        # pre-allocate stuff
        # self.preallocated_ros_array = toRosDType(numpy_dtype=self._dtype,
        #                             is_array=True)()

        pass

    def _close(self):
    
        # called in the close()

        for i in range(len(self._ros_subscribers)):

                if self._ros_subscribers[i] is not None:

                    self._ros_subscribers[i].unregister()
                    