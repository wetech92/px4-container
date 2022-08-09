import setup_path 
import airsim
import time

client = airsim.MultirotorClient()
client.confirmConnection()

pose = client.simGetVehiclePose()

pose.position.x_val = -240.0
pose.position.y_val = -240.0

client.simSetVehiclePose(pose, True, "Typhoon_1")
