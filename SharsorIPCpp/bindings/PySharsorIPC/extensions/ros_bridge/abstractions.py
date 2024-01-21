from abc import ABC, abstractmethod

from typing import List

class RosMessage(ABC):
    
    pass

class RosPublisher(ABC):

    def __init__(self,
                topics: List[str],
                rate: float):
        
        self._topics = topics
        
        self._rate = rate

        self._terminated = False

        self._init_publisher()

    def __del__(self):

        self.shutdown()

    @abstractmethod
    def _init_publisher(self):

        pass
    
    @abstractmethod
    def publish(self, 
            topic: str, 
            message: RosMessage):
        
        pass
    
    @abstractmethod
    def shutdown(self):
        
        if not self._terminated:

            self._terminated = True

class RosSubscriber(ABC):

    def __init__(self,
                topics: List[str],
                rate: float):
        
        self._topics = topics
        
        self._rate = rate

        self._terminated = False

        self._init_subscriber()

    def __del__(self):

        self.shutdown()

    @abstractmethod
    def _init_subscriber(self):

        pass
    
    @abstractmethod
    def subscribe(self, 
            topic: str, 
            callback: RosMessage):
        
        pass
    
    @abstractmethod
    def shutdown(self):
        
        if not self._terminated:

            self._terminated = True
