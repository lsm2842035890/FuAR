import docker

def ip():
    ctn = docker.from_env().containers.get("apollo_dev_lsm")
    return ctn.attrs['NetworkSettings']

if __name__ == '__main__':
    print(ip())