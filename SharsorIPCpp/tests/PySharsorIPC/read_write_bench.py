import unittest
import numpy as np
import time

from SharsorIPCpp.PySharsorIPC import ClientFactory
from SharsorIPCpp.PySharsorIPC import VLevel
from SharsorIPCpp.PySharsorIPC import dtype, toNumpyDType

namespace = "ConnectionTests"

class TestAddFunction(unittest.TestCase):

    def test_init_client(self):

        # Create a client
        client = ClientFactory(basename="SharsorBool",
                               namespace=namespace,
                               verbose=True,
                               vlevel=VLevel.V3,
                               dtype=dtype.Bool)

        client.attach()  # attach to server or wait for it

        # dtype must be consistent
        output = np.zeros((client.getNRows(), client.getNCols()),
                          dtype=toNumpyDType(client.getScalarType()),
                          order='F')

        output_view = output[1:-1, 1:-1]  # (to ensure consistency)

        total_time = 0
        max_time = 0
        iterations = 100

        for i in range(iterations):  # we are connected

            success = False

            # Start the timer
            start_time = time.perf_counter()

            # read the shared tensor (copy)
            success = client.readTensor(output, 0, 0)

            # Stop the timer
            elapsed_time = time.perf_counter() - start_time

            # Update total time and max time
            total_time += elapsed_time
            max_time = max(max_time, elapsed_time)

            # Check the success flag
            if success:
                print("Tensor successfully read:")
                print(output)
                print("View")
                print(output_view)
            else:
                print("Failed to read the tensor.")

            time.sleep(0.1)

        average_time = total_time / iterations

        print(f"Average read time: {average_time:.10f} seconds.")
        print(f"Maximum read time: {max_time:.10f} seconds.")

if __name__ == "__main__":
    unittest.main()
