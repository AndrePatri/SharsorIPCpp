import unittest
import numpy as np

import time

from SharsorIPCpp.PySharsorIPC import ClientFactory
#from SharsorIPCpp.PySharsorIPC import StringTensorClient
from SharsorIPCpp.PySharsorIPC import VLevel
from SharsorIPCpp.PySharsorIPC import dtype, toNumpyDType

namespace = "ConnectionTests"

class TestAddFunction(unittest.TestCase):

#    def test_readwrite_bool(self):

#        # Create a client with the specified parameters
#        client = StringTensorClient(basename="SharedStrTensor",
#                            name_space="ConnectionTests",
#                            verbose=True,
#                            vlevel=VLevel.V3)

#        client.run() # start the client

#        self.assertEqual(5, 5)

#    def test_readwrite_int(self):

#        self.assertEqual(5, 5)

#    def test_readwrite_float(self):

#        self.assertEqual(5, 5)

#    def test_readwrite_double(self):

#        self.assertEqual(5, 5)

#    def test_readwrite_str(self):

#        # client.read_vec()

#        self.assertEqual(5, 5)

    def test_init_client(self):

        # Create a client
        client = ClientFactory(basename = "SharsorBool",
                              namespace = namespace,
                              verbose = True,
                              vlevel = VLevel.V3,
                              dtype=dtype.Bool)

        client.attach() # attach to server or wait for it

        # dtype must be consistent
        output = np.zeros((client.getNRows(),
                        client.getNCols()) ,
                        dtype=toNumpyDType(client.getScalarType()),
                        order = 'F')

        output_view = output[1:-1, 1:-1]
        #(to ensure consistency)

        for i in range(0, 1000): # we are connected

            success = False
            # read the shared tensor (copy)
            success = client.readTensor(output, 0, 0)

            # Check the success flag
            if success:

                print("Tensor successfully read:")

                print(output)

                print("View")
                print(output_view)

            else:

                print("Failed to read the tensor.")

            time.sleep(0.5)

if __name__ == "__main__":

    unittest.main()
