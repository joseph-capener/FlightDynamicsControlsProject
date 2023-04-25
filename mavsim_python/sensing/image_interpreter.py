# Adam Welker   MEEN 674    Winter 2023
#
# image_interpreter.py -- a class that will, 
# given the mask image, find the centroid of the mask and 
# then calculate the view angle 

import numpy as np
import cv2 as cv
from sensing.digital_camera import Simulated_Camera



class ImageInterpreter:

    
    cam = None

    def __init__(self, digital_camera: Simulated_Camera) -> None:
        """
        Function that initializes the class

        :param digital_camera: A simulated_Camera object with signifies the camera
        being used to capture the images passed to the interpreter
        """

        self.cam = digital_camera


    def get_image_centroid(self, mask: np.ndarray) -> np.ndarray:
        """
        Function that given a mask image, returns the centroid location of the image
        notated in pixel indeces [xi, yi]

        :param mask: the mask image to be passed into the function. ndarray should contain 
        only binary values

        :return: an ndarray type in the format (x_c, y_c), where x_c and y_c are the centroid
        locations in pixels within the image frame. returns values -1, -1  if no object is fou d
        """

        moment = cv.moments(mask)

        try:
            cX = int(moment["m10"] / moment["m00"])
            cY = int(moment["m01"] / moment["m00"])

            return np.array([cX, cY])
        
        except:

            return np.array([-1,-1])




    def calculate_LOS_vector(self, centroid_location: np.ndarray, shape: tuple) -> np.ndarray:
        """
        Function that given an image mask will return the Line of Sight (LOS) vector.

        :param centroid_location: an ndarray type in the format (x_c, y_c), where x_c and y_c are the centroid
        locations in pixels within the image frame

        :return: a unit vector expressing the direction from the camera to the object being observed. In format [[x_i, y_i, z_i]]
        """


        center_x = int(shape[0] / 2.)
        center_y = int(shape[1] / 2.)

        dx = self.cam.image_size[0] / shape[0] # change in x per pixel  
        dy = -self.cam.image_size[1] / shape[1] # change in y per pixel 

        
        xi = (centroid_location[0] - center_x) * dx            
        yi = (centroid_location[1] - center_y) * dy     

        zi = np.abs(self.cam.focal_distance)

        return np.array([[xi, yi, zi]]).T / np.linalg.norm(np.array([[xi, yi, zi]]).T)



    def interpret_image(self, mask: np.ndarray) -> np.ndarray:
        """"
        Function that will interpret a mask image and return the LOS vector associated with the 
        object found in the mask

        :param mask: the mask image to be passed into the function. ndarray should contain 
        only binary values

        :return: a unit vector expressing the direction from the camera to the object being observed. In format [[x_i, y_i, z_i]]
        """

        centroid = self.get_image_centroid(mask)
        
        return self.calculate_LOS_vector(centroid,mask.shape)
    


# ====== Main Method ======
# Just tests vectoring on
#  a local PC camera
# =========================
if __name__ == '__main__':

    video = cv.VideoCapture(0) # open a camera

    cam = Simulated_Camera(10,np.array([300,300]),np.array([10,10]))
    interpretor = ImageInterpreter(cam)

    while(True):

        _, frame  = video.read() # read the camera image


        print(frame)
        # mask a mask
        hsv_frame = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

        red_lower = np.array([10, 50, 180])
        red_upper = np.array([40, 255, 255])

        mask = cv.inRange(hsv_frame, red_lower, red_upper)

        # interpret the centroid
        centroid = interpretor.get_image_centroid(mask)

        color_mask = cv.cvtColor(mask,cv.COLOR_GRAY2BGR)

        # map the centroid
        if centroid.item(0) != -1:

            cv.circle(color_mask,centroid,8,[0,0,255],-1)

            print(interpretor.interpret_image(mask))


        cv.imshow("live_video", color_mask) # show the camera feel

        if cv.waitKey(1) & 0xFF == ord('q'): # enable quit key
            break
  

