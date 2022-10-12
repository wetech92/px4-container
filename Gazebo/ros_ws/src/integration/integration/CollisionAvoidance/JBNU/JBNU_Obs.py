import cv2
import numpy as np
from random import *
#import airsim
# from tensorflow.keras.models import load_model
import onnx
import onnxruntime as rt

class JBNU_Collision():
    def __init__(self):
        # AIRSIM DEPENDENT CODES: WILL BE REMOVED
        #self.client = airsim.CarClient()
        #self.client.confirmConnection()
        
        #self.model_pretrained = load_model('/root/ros_ws/src/integration/integration/CollisionAvoidance/JBNU/Best_feed_forward.h5')
        model_pretrained = onnx.load('/root/ros_ws/src/integration/integration/CollisionAvoidance/JBNU/feedforward.onnx')
        onnx.checker.check_model(model_pretrained)
        sess = rt.InferenceSession(model_pretrained.SerializeToString(),providers=rt.get_available_providers())


        print("initialJBNU")
        
    def CA(self, Image):
        """
        responses = self.client.simGetImages([airsim.ImageRequest("", airsim.ImageType.DepthPerspective,  True,False)]) 
        response = responses[0]
        if ((responses[0].width != 0 or responses[0].height != 0)):
            img1d = response.image_data_float
            img2d = np.reshape(img1d,(response.height, response.width)) 
        else:
            print("Something bad happened! Restting AirSim!")
            img2d = np.ones(640, 480)
        """
        img2d = Image

        depth_8bit_lerped = np.interp(img2d, (0.0, 10), (0, 255))
        image=cv2.applyColorMap(cv2.convertScaleAbs(depth_8bit_lerped,alpha=1),cv2.COLORMAP_JET)
        image=cv2.resize(image,(200,200),cv2.INTER_AREA)
        infer = self.model_pretrained.predict(image, batch_size=1)
        vx = infer[0][0]
        vy = 0.0
        vz = infer[0][1]
        vyaw = infer[0][2]
        return vx,vy,vz,vyaw

        

if __name__ == '__main__':
    tensor = JBNU_Collision()
    vx,vy,vz,vyaw =tensor.main()
    print(vx,vy,vz,vyaw)
 
