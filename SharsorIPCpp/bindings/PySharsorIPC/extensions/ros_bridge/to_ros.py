from SharsorIPCpp.PySharsorIPC import ClientFactory
from SharsorIPCpp.PySharsorIPC import StringTensorClient

from typing import Union

from SharsorIPCpp.PySharsor.extensions.ros_bridge.defs import NamingConventions

class ToRos():

    # Atomic bridge element to forward data from SharsorIPCpp or PySharsorIPCpp
    # over topic. Can be useful when developing distributed architectures or when 
    # one needs to implement remote debugging features 

    # Given a client object (either a normal client or a string client), 
    # this object creates a publisher to a number of conventional topics to
    # stream both data and meta data associated with Sharsor.

    def __init__(self,
                client: Union[ClientFactory, 
                            StringTensorClient],
                rate: float):

        self._client = client

        self._rate = rate

        self._naming_conv = NamingConventions()

