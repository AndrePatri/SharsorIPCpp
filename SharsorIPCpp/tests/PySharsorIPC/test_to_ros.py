from SharsorIPCpp.PySharsor.extensions.ros_bridge.to_ros import *
from SharsorIPCpp.PySharsor.wrappers.shared_data_view import *

from SharsorIPCpp.PySharsorIPC import *

import numpy as np

import time
from perf_sleep.pyperfsleep import PerfSleep

import rospy

import os

# Function to set CPU affinity
def set_affinity(cores):
    try:
        os.sched_setaffinity(0, cores)
        print(f"Set CPU affinity to cores: {cores}")
    except Exception as e:
        print(f"Error setting CPU affinity: {e}")

order = 'C'

client = SharedDataView(namespace = "Prova",
            basename = "ToRosTest",
            is_server = False, 
            verbose = True, 
            vlevel = VLevel.V3,
            dtype= dtype.Float,
            safe = False)
client.run()

update_dt = 0.001
start_time = time.perf_counter() 
start_time = 0.0
elapsed_time = 0.0
actual_loop_dt = 0.0

time_to_sleep_ns = 0
debug = False

perf_timer = PerfSleep()

namespace = 'Shared2RosBridge'
rospy.init_node(namespace)
# loop_rate = rospy.Rate(int(1/update_dt))  # Set an initial loop rate, e.g., 1000 Hz

bridge = ToRos(client.shared_mem,
        queue_size = 1,
        ros_backend = "ros1")

bridge.run(latch=True)

msg = f"Will try to run the bridge at {1/update_dt} Hz."
Journal.log("test_to_ros.py",
            "",
            msg,
            LogType.INFO,
            throw_when_excep = True)

try:

    set_affinity([15])

    while not rospy.is_shutdown():

        start_time = time.perf_counter() 

        # server.numpy_view[:, :] = np.random.rand(server.n_rows, server.n_cols)

        bridge.update()

        elapsed_time = time.perf_counter() - start_time

        time_to_sleep_ns = int((update_dt - elapsed_time) * 1e+9) # [ns]

        if time_to_sleep_ns < 0:

            warning = f"Could not match desired update dt of {update_dt} s. " + \
                f"Elapsed time to update {elapsed_time}."

            Journal.log("test_to_ros.py",
                        "",
                        warning,
                        LogType.WARN,
                        throw_when_excep = True)

        perf_timer.clock_sleep(time_to_sleep_ns) 

        # loop_rate.sleep()

        actual_loop_dt = time.perf_counter() - start_time

        if debug:

            print(f"Actual loop dt {actual_loop_dt} s.")

except KeyboardInterrupt:
    print("\nCtrl+C pressed. Exiting the loop.")




