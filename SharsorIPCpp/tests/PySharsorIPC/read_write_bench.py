import unittest
import numpy as np
import torch

import time

from SharsorIPCpp.PySharsorIPC import ClientFactory, PyClientBoolRowMaj
from SharsorIPCpp.PySharsorIPC import VLevel
from SharsorIPCpp.PySharsorIPC import RowMajor, ColMajor
from SharsorIPCpp.PySharsorIPC import dtype, toNumpyDType

namespace = "ConnectionTests"

class TestAddFunction(unittest.TestCase):

    def test_client_wrapper(self):

        # Create a client
        client = ClientFactory(basename="SharsorBool",
                               namespace=namespace,
                               verbose=True,
                               vlevel=VLevel.V3,
                               dtype=dtype.Bool,
                               layout=RowMajor)

        client.attach()  # attach to server or wait for it

        # dtype must be consistent
        output_torch = torch.zeros((client.getNRows(), client.getNCols()),
                                    dtype=torch.bool)

        output_numpy = np.zeros((client.getNRows(), client.getNCols()),
                        dtype=np.float,
                        order="C")

        output_view = output_numpy[1:-1, 1:-1]  # (to ensure consistency)

        print("Output C-contiguous:", output_torch.is_contiguous())
        print("output_numpy C-contiguous:", output_numpy.flags.c_contiguous)
        print("output_view C-contiguous:", output_view.flags.c_contiguous)
        print("output_view is F-contiguous:", output_view.flags.f_contiguous)

        total_time = 0
        max_time = 0
        iterations = 1000000

        for i in range(iterations):  # we are connected

            success = False

            # Start the timer
            start_time = time.perf_counter()

            # read the shared tensor (copy)
            success = client.readTensor(output_numpy, 0, 0)

            # Stop the timer
            elapsed_time = time.perf_counter() - start_time

            # Update total time and max time
            total_time += elapsed_time
            max_time = max(max_time, elapsed_time)

            # Check the success flag
            if success:
#                print("Tensor successfully read.")
#                print("Torch:")
#                print(output_torch)
#                print("Numpy:")
#                print(output_numpy)
#                print("Numpy view")
#                print(output_view)
                a = 1
            else:
                print("Failed to read the tensor.")

#            time.sleep(0.1)

        average_time = total_time / iterations

        print(f"Average read time: {average_time:.10f} seconds.")
        print(f"Maximum read time: {max_time:.10f} seconds.")

#        def test_client_wrapper(self):

#            client = PyClientBoolRowMaj("SharsorBool",
#                                        namespace,
#                                        True,
#                                        VLevel.V3)

#            client.attach()  # attach to server or wait for it

#            # dtype must be consistent
#            output_torch = torch.zeros((client.getNRows(), client.getNCols()),
#                                        dtype=torch.bool)

#            output_numpy = np.zeros((client.getNRows(), client.getNCols()),
#                            dtype=np.bool,
#                            order="C")

#            output_view = output_numpy[1:-1, 1:-1]  # (to ensure consistency)

#            print("Output C-contiguous:", output_torch.is_contiguous())
#            print("output_numpy C-contiguous:", output_numpy.flags.c_contiguous)
#            print("output_view C-contiguous:", output_view.flags.c_contiguous)
#            print("output_view is F-contiguous:", output_view.flags.f_contiguous)

#            total_time = 0
#            max_time = 0
#            iterations = 1000000

#            for i in range(iterations):  # we are connected

#                success = False

#                # Start the timer
#                start_time = time.perf_counter()

#                # read the shared tensor (copy)
#                success = client.readTensor(output_numpy, 0, 0)

#                # Stop the timer
#                elapsed_time = time.perf_counter() - start_time

#                # Update total time and max time
#                total_time += elapsed_time
#                max_time = max(max_time, elapsed_time)

#                # Check the success flag
#                if success:
##                    print("Tensor successfully read.")
##                    print("Torch:")
##                    print(output_torch)
##                    print("Numpy:")
##                    print(output_numpy)
##                    print("Numpy view")
##                    print(output_view)
#                    a = 1
#                else:
#                    print("Failed to read the tensor.")

##                time.sleep(0.1)

#            average_time = total_time / iterations

#            print(f"Average read time: {average_time:.10f} seconds.")
#            print(f"Maximum read time: {max_time:.10f} seconds.")


if __name__ == "__main__":
    unittest.main()
