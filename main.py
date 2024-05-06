import Camera
import RadioGps
import TrackingControl
import WebServer


from multiprocessing import Process, Manager
import time
import redis
from db import RedisClient

r = redis.Redis()
client = RedisClient(r)

PERSISTENT_FILENAME = "db.txt"

PROCESSES = [
    Camera,
    #RadioGps,
    TrackingControl
   # WebServer,
]

if __name__ == '__main__':
    manager = Manager()
    client.set_initial("stop_surf", False)
    d = manager.dict()
    d["stop"] = False
    #client.load(PERSISTENT_FILENAME)
    
    process_list = []
    for p in PROCESSES:
        process_list.append(Process(target=p.main, args=(d,)))
    for p in process_list:
        p.start()