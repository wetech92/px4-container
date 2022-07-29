import setup_path
import airsim
import random
import time
import os
import threading

asset_name = 'Birch_01'

def genObj(low,high):
    objList = list()
    for i in range(low,high):
        client = airsim.VehicleClient(ip = str(os.environ['simhost']), port=41451)
        desired_name = f"{asset_name}_spawna_{i}"
        scale = airsim.Vector3r(random.uniform(0.5,2.0), random.uniform(0.5,2.0), random.uniform(0.5,2.0))
        pose = airsim.Pose(position_val=airsim.Vector3r(random.randrange(-250,250), random.randrange(-250,250), 0.0))

        obj_name = client.simSpawnObject(desired_name, asset_name, pose, scale, False)
        objList.append(obj_name)
        all_objects = client.simListSceneObjects()

        if obj_name not in all_objects:
            print(f"Object {obj_name} not present!")

t1 = threading.Thread(target=genObj, args=(1, 400))
t2 = threading.Thread(target=genObj, args=(401, 800))
t3 = threading.Thread(target=genObj, args=(801, 1200))
t4 = threading.Thread(target=genObj, args=(1201, 1600))
t5 = threading.Thread(target=genObj, args=(1601, 2000))

t1.start()
t2.start()
t3.start()
t4.start()
t5.start()

t1.join()
t2.join()
t3.join()
t4.join()
t5.join()

client = airsim.VehicleClient(ip = str(os.environ['simhost']), port=41451)
test = client.simListSceneObjects(name_regex=f"{asset_name}_spawna_.*")
print(f"Total of {len(test)} objects created!")
