import RRT
import numpy as np
import matplotlib.pyplot as plt
import cv2

StartPoint = np.array([[100], [100]])
GoalPoint = np.array([[4900], [4900]])

# StartPoint = np.array([[0], [0]])
# GoalPoint = np.array([[4999], [4999]])


RRTV = RRT.RRT()


RawImage = cv2.imread("/root/ros_ws/src/integration/integration/PathPlanning/Map/Map.png", cv2.IMREAD_GRAYSCALE)
# Image = cv2.imread("/root/ros_ws/src/integration/integration/PathPlanning/Map/Map.png", cv2.IMREAD_GRAYSCALE)

Image = np.uint8(np.uint8((255 - RawImage)/ 255))
Image = cv2.flip(Image, 0)
# Image = cv2.rotate(Image, cv2.ROTATE_90_CLOCKWISE)
Planned = RRTV.PathPlanning(Image, StartPoint, GoalPoint)
PlannedX = Planned[0]
PlannedY = Planned[1]
MaxPlannnedIndex = len(PlannedX) - 1

print(len(PlannedX))
print(PlannedX)

bgd = plt.imread("/root/ros_ws/src/integration/integration/PathPlanning/Map/Map.png")
plt.imshow(bgd,zorder=0, extent=[0, 5000, 0, 5000])

plt.plot(PlannedX,PlannedY)
plt.savefig('plotting.png')


# cv2.imshow('Image',RawImage)
# cv2.waitKey(0)
# cv2.destroyAllWindows()