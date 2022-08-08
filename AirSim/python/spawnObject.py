import setup_path
import airsim
import argparse
import random
import time
import os
import numpy as np
import threading

def genObj(asset_name,region,low,high):
    objList = list()
    for i in range(low,high):
        client = airsim.VehicleClient()
        desired_name = f"{asset_name}_airsim_{i}"
        scale = airsim.Vector3r(random.uniform(1.0,2.0), random.uniform(1.0,2.0), random.uniform(1.0,2.0))

        posX = random.randrange(-region[0],region[0])
        posY = random.randrange(-region[1],region[1])
        posZ = 0.3
        dist1 = np.linalg.norm(np.array((posX,posY)) - np.array((-240,-240)))
        dist2 = np.linalg.norm(np.array((posX,posY)) - np.array((240,240)))

        # Reroll until flight area is secured
        while dist1 <= 2.5 or dist2 <= 2.5:
            posX = random.randrange(-region[0],region[0])
            posY = random.randrange(-region[1],region[1])
            distOrigin = np.linalg.norm(np.array((posX,posY)) - np.array((-240,-240)))

        pose = airsim.Pose(position_val=airsim.Vector3r(posX,posY,posZ))

        obj_name = client.simSpawnObject(desired_name, asset_name, pose, scale, False)
        objList.append(obj_name)
        all_objects = client.simListSceneObjects()

        if obj_name not in all_objects:
            print(f"Object {obj_name} not present!")

def main():
    parser = argparse.ArgumentParser(
        description='Generate an available asset in AirSim environment with given parameters')
    optional = parser._action_groups.pop()
    required = parser.add_argument_group('Required')
    required.add_argument("-a", "--asset", dest="asset_name",
                          help="Unreal asset to create (string)", metavar="ASSET")
    required.add_argument("-r", "--region", dest="region",nargs='+',type=float,
                          help="Target region to create objects ([x,y])", metavar="REGION")
    optional.add_argument("-n", "--number", dest="output_file",
                          help="Number of assets to create (int). If not set, script will create only 1 of it",
                          metavar="NUM", default=1)

    args = parser.parse_args()

    print(f"Start creating {args.asset_name}")
    startTime = time.time()

    n = 120
    t1 = threading.Thread(target=genObj, args=(args.asset_name,args.region,1, n))
    t2 = threading.Thread(target=genObj, args=(args.asset_name,args.region,n+1, 2*n))
    t3 = threading.Thread(target=genObj, args=(args.asset_name,args.region,2*n+1, 3*n))
    t4 = threading.Thread(target=genObj, args=(args.asset_name,args.region,3*n+1, 4*n))
    t5 = threading.Thread(target=genObj, args=(args.asset_name,args.region,4*n+1, 5*n))

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

    client = airsim.VehicleClient()
    objNum = client.simListSceneObjects(name_regex=f"{args.asset_name}_airsim_.*")
    ellapsed = round(time.time() - startTime)

    print(f"Total of {len(objNum)} objects created!")
    print(f"This took {ellapsed} seconds")

if __name__ == "__main__":
    main()
