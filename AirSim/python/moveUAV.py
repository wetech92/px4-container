import setup_path 
import airsim
import time

client = airsim.MultirotorClient()
client.confirmConnection()

pose = client.simGetVehiclePose()

# teleport the drone + 10 meters in x-direction
pose.position.x_val -= 240
pose.position.y_val -= 240

client.simSetVehiclePose(pose, True, "Typhoon_1")
