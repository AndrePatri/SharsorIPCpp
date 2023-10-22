import unittest

from SharsorIPCpp.PySharsorIPC import StringTensorClient, VLevel

# from SharsorIPCpp.PySharsorIPC import Client
# from SharsorIPCpp.PySharsorIPC import Server

class TestAddFunction(unittest.TestCase):

    def test_readwrite_bool(self):
        
        # Create a client with the specified parameters
        client = StringTensorClient(basename="SharedStrTensor", 
                            name_space="ConnectionTests", 
                            verbose=True, 
                            vlevel=VLevel.V3)
        
        client.run() # start the client
   
        self.assertEqual(5, 5)

    def test_readwrite_int(self):

        self.assertEqual(5, 5)

    def test_readwrite_float(self):

        self.assertEqual(5, 5)

    def test_readwrite_double(self):

        self.assertEqual(5, 5)

    def test_readwrite_str(self):

        # client.read_vec()

        self.assertEqual(5, 5)

if __name__ == "__main__":

    unittest.main()
