from SharsorIPCpp.PySharsorIPC import ClientFactory
from SharsorIPCpp.PySharsorIPC import StringTensorClient

from typing import Union

from SharsorIPCpp.PySharsorIPC.extensions.ros_bridge.defs import NamingConventions

class FromRos():

    # Atomic bridge element to forward data from conventional Ros Topics 
    # on shared memory thorugh PySharsorIPCpp. Can be useful when developing
    # distributed architectures or when one needs to implement remote debugging features 

    # Given a basename and namespace, this object creates a suitable SharsorIPCpp server
    # which is created reading metadata on the conventional topics and updated with data 
    # streaming on topics in real time

    def __init__(self,
                client: Union[ClientFactory, 
                            StringTensorClient],
                rate: float):

        self._client = client

        self._rate = rate

        self._naming_conv = NamingConventions()