import docker

class ContainerMonitor(object):
    def __init__(self):
        self._client  = docker.from_env()

    def get_all_running_containers(self, subname):
        controllers = self._get_all_containers_with_status(subname,"running")
        return controllers
    
    def _get_all_containers_with_status(self,subname,status):
        containers = self._client.containers.list(all=True)
        controllers = []
        for container in containers:
            if subname in container.name and status in container.status:
                controllers.append(container.name)
        return controllers
    
    def stop_all_containers_except(self, whitelist):
        containers = self._client.containers.list(all=True)
        for container in containers:
            if container.name not in whitelist:
                container.stop()
                container.remove()
