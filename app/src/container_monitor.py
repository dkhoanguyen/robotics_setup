import docker
import time

class ContainerMonitor(object):
    def __init__(self):
        self._client  = docker.from_env()

    def get_all_containers(self):
        pass