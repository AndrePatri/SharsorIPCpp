import rclpy
from rclpy.qos import QoSProfile
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from rclpy.qos import ReliabilityPolicy, DurabilityPolicy, HistoryPolicy, LivelinessPolicy

from std_msgs.msg import Bool, Int32, Float32, Float64
from std_msgs.msg import Int32MultiArray, Float32MultiArray, Float64MultiArray

from SharsorIPCpp.PySharsor.extensions.ros_bridge.abstractions import RosPublisher
from SharsorIPCpp.PySharsor.extensions.ros_bridge.abstractions import RosSubscriber

import numpy as np

def toRosDType(numpy_dtype, is_array=False):

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

class Ros2Publisher(RosPublisher):

    def __init__(self,
                node: rclpy.node.Node,
                n_rows: int, 
                n_cols: int,
                basename: str,
                namespace: str = "",
                queue_size: int = 1, # by default only read latest msg
                dtype = np.float32):
        
        self._node = node

        self._qos_settings = QoSProfile(
                    reliability=ReliabilityPolicy.RELIABLE, # BEST_EFFORT
                    durability=DurabilityPolicy.TRANSIENT_LOCAL, # VOLATILE
                    history=HistoryPolicy.KEEP_LAST, # KEEP_ALL
                    depth=queue_size,  # Number of samples to keep if KEEP_LAST is used
                    liveliness=LivelinessPolicy.AUTOMATIC,
                    # deadline=1000000000,  # [ns]
                    # partition='my_partition' # useful to isolate communications
                    )

        super().__init__(n_rows=n_rows, 
                n_cols=n_cols,
                basename=basename,
                namespace=namespace,
                queue_size=queue_size, # by default only read latest msg
                dtype=dtype)

        self._to_list_first = False # override default

        self._use_np_directly = False

    def _create_publisher(self,
                    name: str, 
                    dtype, 
                    queue_size: int,
                    is_array = False,
                    latch = True):

        publisher = self._node.create_publisher(msg_type=toRosDType(dtype, is_array),
                                    topic=name,
                                    qos_profile=self._qos_settings)
        
        return publisher

    def _prerun(self):
        
        # pre-allocate stuff

        self.preallocated_ros_array = toRosDType(
            numpy_dtype=self._dtype, is_array=True)()

        self.preallocated_ros_array.data = self.preallocated_np_array.flatten()

    def _close(self):

        # called in the close()

        if self.publisher is not None:

            self.publisher.destroy()

class Ros2Subscriber(Node):

    def __init__(self,
                node: rclpy.node.Node,
                basename: str,
                namespace: str = "",
                queue_size: int = 1):

        self._node = node
        
        super().__init__(basename=basename,
                namespace=namespace,
                queue_size=queue_size)

        self._use_np_directly = False
        
    def _create_subscriber(self,
                    name: str, 
                    dtype, 
                    callback, 
                    callback_args = None,
                    queue_size: int = None,
                    is_array = False):

        qos_settings = QoSProfile(depth=queue_size)

        subscriber = self._node.create_subscription(msg_type = toRosDType(dtype, is_array),
                        topic=name,
                        callback=callback,
                        qos_settings=qos_settings,
                        raw=False
                        )

    def postrun(self):

        # pre-allocate stuff

        self.preallocated_ros_array = toRosDType(
            numpy_dtype=self._dtype, is_array=True)()

    def close(self):

        # called in the close()
        if self.subscription is not None:

            self.subscription.destroy()
