"""obo_controller_v1 controller."""

# Imports
from controller import Robot, Camera, Display,DistanceSensor, Compass, TouchSensor
import cv2
import numpy as np

#Starting opencv window
cv2.startWindowThread()
cv2.namedWindow("preview")

if __name__ == "__main__":

    # create the Robot instance.
    robot = Robot()
    
    timestep = 64

    # Max Speed of motors
    max_speed = 5
    left_speed = max_speed
    right_speed = max_speed
    
    # Motors
    right_motor1 = robot.getDevice("motor_RB")
    right_motor2 = robot.getDevice('motor_RF')
    left_motor1 = robot.getDevice('motor_LB')
    left_motor2 = robot.getDevice('motor_LF')
    
    # Devices of Front Arm
    pole_motor = robot.getDevice("pole motor")
    vertical_motor = robot.getDevice("vertical motor")
    horizontal_R_motor = robot.getDevice("horizontal motorR")
    horizontal_L_motor = robot.getDevice("horizontal motorL")
    vacL = robot.getDevice("vacuum gripperL")
    vacR = robot.getDevice("vacuum gripperR")
    
    # Initializing Touch Sensor
    bumper = robot.getDevice("touch sensor")
    bumper.enable(64)
    
    # Initializing motors
    left_motor1.setPosition(float('inf'))
    left_motor1.setVelocity(0.0)
    left_motor2.setPosition(float('inf'))
    left_motor2.setVelocity(0.0)
    right_motor1.setPosition(float('inf'))
    right_motor1.setVelocity(0.0)
    right_motor2.setPosition(float('inf'))
    right_motor2.setVelocity(0.0)
    
    # Initializing Cameras
    front_cam = robot.getDevice("front camera")
    front_cam.enable(64)
    top_cam = robot.getDevice("top camera")
    top_cam.enable(64)
    
    # Initializing Distance Sensors
    distance_sensor_FL = robot.getDevice("distance sensorFL")
    distance_sensor_FL.enable(64)
    distance_sensor_gripper = robot.getDevice("gripper_distance_sensor")
    distance_sensor_gripper.enable(64)
    
    # Initializing Compass
    compass = robot.getDevice("compass")
    compass.enable(64)
    
    # Initializing Front Arm
    horizontal_R_motor.setVelocity(1)
    horizontal_L_motor.setVelocity(1)
    vertical_motor.setVelocity(2)
    pole_motor.setVelocity(2)
    pole_motor.setPosition(0) # 0 is initial 3.14 is back

    # Starting Delay
    robot.step(64*20)
    
    vertical_motor.setPosition(0.1)
    horizontal_R_motor.setPosition(-0.02)
    horizontal_L_motor.setPosition(0.06)

    
    # Variables
    i=0
    skipper = 0
    wall_hitter = 0
    status = 0
    balls_put = 0
    
    # Main Loop
    
    while robot.step(timestep) != -1:
        
        print("i == ",i) # printing the counter

        # Reading Distance of Distance sensors
        distanceFL = 400 - round(distance_sensor_FL.getValue(),0)
        print("distance = " ,distanceFL)
        
        distancegripper = round(distance_sensor_gripper.getValue(),0)
        print("Gripper distance = " ,distancegripper)

        # Reading Compass Measurements
        compassX = round(compass.getValues()[0],3)
        compassY = round(compass.getValues()[1],3)
        #print("compass x= ",compassX," y = ",compassY)
        
        # Reading Touch sensor values
        bumperReading = round(bumper.getValues()[1]*100,0)
        #print("Bumper = ",bumperReading)
        
        
        if(status == 0):    # Red ball follow 
            # Red ball detction and follow
            image = front_cam.getImage()
            
            image_array = np.frombuffer(image, dtype=np.uint8).reshape((front_cam.getHeight(), front_cam.getWidth(), 4))
        
            hsv_image = cv2.cvtColor(image_array, cv2.COLOR_RGB2HSV)
        
            # Define the lower and upper bounds for the red color 
            lower_red = np.array([100, 150, 0])
            upper_red = np.array([250, 230, 255])
            
        
            # Create a mask to isolate red pixels
            mask1 = cv2.inRange(hsv_image, lower_red, upper_red)
            
            # shades of red
            lower_red = np.array([160, 100, 100])
            upper_red = np.array([179, 255, 255])
    
            mask2 = cv2.inRange(hsv_image, lower_red, upper_red)
    
            red_mask = cv2.bitwise_or(mask1, mask2)
    
            red_ball = cv2.bitwise_and(image_array, image_array, mask=red_mask)
        
            # Find contours of the red object
            contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
            for contour in contours:
                x, y, w, h = cv2.boundingRect(contour)
                cv2.circle(red_ball, (int(x + w/2), int(y + h/2)), int((w + h)/4), (0, 255, 0), 2)
                
                # Largest Contour
                largest_contour = max(contours, key=cv2.contourArea)
                
                M = cv2.moments(largest_contour)
                
                if M['m00'] > 0:
                
                    cx = int(M['m10'] / M['m00'])  # X-coordinate of the object's centroid
                    cy = int(M['m01'] / M['m00'])  # Y-coordinate of the object's centroid
                    
            print("large x = ",cx)
            print("large y = ",cy)
                
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(red_ball, 'red balls: ' + str(len(contours)), (20, 20), font, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
        
            rgb_image = cv2.cvtColor(hsv_image, cv2.COLOR_HSV2RGB)
        
            #cv2.imshow("name",rgb_image)
            cv2.namedWindow("red image", cv2.WINDOW_NORMAL)        # Create window with freedom of dimensions
            cv2.resizeWindow("red image", 300, 300)   
            cv2.imshow("red image",red_ball)
            
    
            if(cx<300 and cy < 500):
                print("ball on left")
                left_motor1.setVelocity(1)
                left_motor2.setVelocity(1)
                right_motor1.setVelocity(10)
                right_motor2.setVelocity(10)
            elif(cx >340 and cy < 500):
                print("ball on right")
                left_motor1.setVelocity(10)
                left_motor2.setVelocity(10)
                right_motor1.setVelocity(1)
                right_motor2.setVelocity(1)
            elif(cy > 500):
                left_motor1.setVelocity(0)
                left_motor2.setVelocity(0)
                right_motor1.setVelocity(0)
                right_motor2.setVelocity(0)
                if(status == 0):
                    print("Grip")
                    #horizontal_R_motor.setPosition(-0.05)
                    #horizontal_L_motor.setPosition(0.1)
                    robot.step(256)
                    vertical_motor.setPosition(0.05)
                    robot.step(256)
                    
                    horizontal_R_motor.setPosition(0.03)
                    horizontal_L_motor.setPosition(-0.02)
           
                    robot.step(64*10)
                    vacL.turnOn()
                    vacR.turnOn()
                    vertical_motor.setVelocity(2)
                    vertical_motor.setPosition(0.3)
                    robot.step(64*10)
                    pole_motor.setPosition(3.14)
                    status = 1
                    
                    left_motor1.setVelocity(left_speed)
                    left_motor2.setVelocity(left_speed)
                    right_motor1.setVelocity(0)
                    right_motor2.setVelocity(0)
                    
                    robot.step(64*40)
                    print("pole position set")
                    pole_motor.setPosition(3.14)
                    left_motor1.setVelocity(0)
                    left_motor2.setVelocity(0)
                    right_motor1.setVelocity(0)
                    right_motor2.setVelocity(0)
                
                
            else:
                print("ball on mid")
                left_motor1.setVelocity(left_speed)
                left_motor2.setVelocity(left_speed)
                right_motor1.setVelocity(right_speed)
                right_motor2.setVelocity(right_speed)
        
        
        elif(status == 1): # Turning towards basckets
            print("Turning to East")
            left_motor1.setVelocity(left_speed)
            left_motor2.setVelocity(left_speed)
            right_motor1.setVelocity(0)
            right_motor2.setVelocity(0)
            if(compassX == -1):
                left_motor1.setVelocity(0)
                left_motor2.setVelocity(0)
                right_motor1.setVelocity(0)
                right_motor2.setVelocity(0)
                status = 2
                print("Robot is on East")
            
        elif(status ==2): # Go towards basckets
            left_motor1.setVelocity(left_speed)
            left_motor2.setVelocity(left_speed)
            right_motor1.setVelocity(right_speed)
            right_motor2.setVelocity(right_speed)
            if(distanceFL <= 150):
                skipper = skipper + 1
                if(skipper >= 5):
                    left_motor1.setVelocity(0)
                    left_motor2.setVelocity(0)
                    right_motor1.setVelocity(0)
                    right_motor2.setVelocity(0)
                    print("Robot stopped near wall")
                    status = 3
                    skipper = 0
            else:
                skipper = 0
                
        
        
        elif(status == 3): # Bascket white follow
            
            # White basket detection
            image = top_cam.getImage()
        
            image_array = np.frombuffer(image, dtype=np.uint8).reshape((top_cam.getHeight(), top_cam.getWidth(), 4))
        
            #bgr_image = cv2.cvtColor(image_array, cv2.COLOR_RGB2BGR)
            
            hsv_image = cv2.cvtColor(image_array, cv2.COLOR_RGB2HSV)
        
            # Define the lower and upper bounds for the white colour
            lower_white = np.array([0, 0, 200])
            upper_white = np.array([0, 255, 255])
        
            mask1 = cv2.inRange(hsv_image, lower_white, upper_white)
            
            # Shades of white
            lower_white = np.array([0, 0, 200])
            upper_white = np.array([0, 0, 255])
    
            mask2 = cv2.inRange(hsv_image, lower_white, upper_white)
    
            white_mask = cv2.bitwise_or(mask1, mask2)
    
            white_ball = cv2.bitwise_and(image_array, image_array, mask=white_mask)
        
            # Find contours of the white object
            contours, _ = cv2.findContours(white_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
            area_array=[]
            for contour in contours:
                x, y, w, h = cv2.boundingRect(contour)
                cv2.circle(white_ball, (int(x + w/2), int(y + h/2)), int((w + h)/4), (0, 255, 0), 2)
                
                largest_contour = max(contours, key=cv2.contourArea)
                #print(largest_contour)
                
                M = cv2.moments(largest_contour)
                
                if M['m00'] > 0:
                
                    cx = int(M['m10'] / M['m00'])  # X-coordinate of the object's centroid
                    cy = int(M['m01'] / M['m00'])  # Y-coordinate of the object's centroid
                    
            print("large x = ",cx)
            print("large y = ",cy)
            
            
                
            if(len(area_array) != 0):
                print(max(area_array))
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(white_ball, 'White balls: ' + str(len(area_array)), (20, 20), font, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
        
            rgb_image = cv2.cvtColor(hsv_image, cv2.COLOR_HSV2RGB)
        
            #custom_view.setImage(rgb_image.tobytes())
            
            #display.setImage(red_frame)
             
            
            #cv2.imshow("name",rgb_image)
            cv2.namedWindow("red image", cv2.WINDOW_NORMAL)        # Create window with freedom of dimensions
            cv2.resizeWindow("red image", 300, 300)   
            cv2.imshow("red image",white_ball)
        
            if(cx<300 and cy < 500):
                print("ball on left")
                left_motor1.setVelocity(0)
                left_motor2.setVelocity(0)
                right_motor1.setVelocity(10)
                right_motor2.setVelocity(10)
            elif(cx >340 and cy < 500):
                print("ball on right")
                left_motor1.setVelocity(left_speed)
                left_motor2.setVelocity(left_speed)
                right_motor1.setVelocity(0)
                right_motor2.setVelocity(0)
            elif(cy > 600):
                left_motor1.setVelocity(0)
                left_motor2.setVelocity(0)
                right_motor1.setVelocity(0)
                right_motor2.setVelocity(0)
                
                status = 4
                vertical_motor.setPosition(0.6)
                robot.step(256)
                pole_motor.setPosition(-1)
                print("UnGrip")
                balls_put = balls_put + 1
                temp1 = 0
                gripperSensorFlag = 0
                prevGripper = 30
                while(temp1<30):
                    distancegripper = round(distance_sensor_gripper.getValue(),0)
                    print("Gripper distance = " ,distancegripper)
                    
                    if(distancegripper < 30 and prevGripper == 30):
                        if(gripperSensorFlag == 1):
                            break
                        gripperSensorFlag = 1
                    robot.step(64)
                    temp1 = temp1+1
                    prevGripper = distancegripper
                
                horizontal_R_motor.setPosition(-0.05)
                horizontal_L_motor.setPosition(0.1)
                vacL.turnOff()
                vacR.turnOff()
                pole_motor.setPosition(0)
                    
                
                
            else:
                print("ball on mid")
                left_motor1.setVelocity(left_speed)
                left_motor2.setVelocity(left_speed)
                right_motor1.setVelocity(right_speed)
                right_motor2.setVelocity(right_speed)
        
        elif(status == 4): # Reverse
            left_motor1.setVelocity(-10)
            left_motor2.setVelocity(-10)
            right_motor1.setVelocity(-10)
            right_motor2.setVelocity(-10)
            robot.step(64*10)
            status = 5
        
        elif(status == 5): # Turning towards mid
            print("Turning to West")
            left_motor1.setVelocity(left_speed)
            left_motor2.setVelocity(left_speed)
            right_motor1.setVelocity(0)
            right_motor2.setVelocity(0)
            if(compassX == 1):
                left_motor1.setVelocity(0)
                left_motor2.setVelocity(0)
                right_motor1.setVelocity(0)
                right_motor2.setVelocity(0)
                if(balls_put < 6):
                    status = 0
                else:
                    status = 10
                print("Robot is on West")
        
        
        elif(status == 10): # Blue ball follow
        
            # Blue ball detection part
            image = front_cam.getImage()
        
            image_array = np.frombuffer(image, dtype=np.uint8).reshape((front_cam.getHeight(), front_cam.getWidth(), 4))
        
            hsv_image = cv2.cvtColor(image_array, cv2.COLOR_RGB2HSV)
        
            # Define the lower and upper bounds for the blue color 
            lower_blue = np.array([10, 150, 0])
            upper_blue = np.array([15, 200, 200])
        
            mask1 = cv2.inRange(hsv_image, lower_blue, upper_blue)
            
            # Shades of blue
            lower_blue_s = np.array([10, 150, 0])
            upper_blue_s = np.array([15, 200, 200])
    
            mask2 = cv2.inRange(hsv_image, lower_blue_s, upper_blue_s)
    
            blue_mask = cv2.bitwise_or(mask1, mask2)
    
            blue_ball = cv2.bitwise_and(image_array, image_array, mask=blue_mask)
        
            contours, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
            for contour in contours:
                x, y, w, h = cv2.boundingRect(contour)
                cv2.circle(blue_ball, (int(x + w/2), int(y + h/2)), int((w + h)/4), (0, 255, 0), 2)
                
                largest_contour = max(contours, key=cv2.contourArea)
                
                M = cv2.moments(largest_contour)
                
                if M['m00'] > 0:
                
                    cx = int(M['m10'] / M['m00'])  # X-coordinate of the object's centroid
                    cy = int(M['m01'] / M['m00'])  # Y-coordinate of the object's centroid
                    
            print("large x = ",cx)
            print("large y = ",cy)
                
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(blue_ball, 'blue balls: ' + str(len(contours)), (20, 20), font, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
        
            rgb_image = cv2.cvtColor(hsv_image, cv2.COLOR_HSV2RGB)
        
            #cv2.imshow("name",rgb_image)
            cv2.namedWindow("red image", cv2.WINDOW_NORMAL)        # Create window with freedom of dimensions
            cv2.resizeWindow("red image", 300, 300)   
            cv2.imshow("red image",blue_ball)
            
    
            if(cx<300 and cy < 450):
                print("ball on left")
                left_motor1.setVelocity(1)
                left_motor2.setVelocity(1)
                right_motor1.setVelocity(10)
                right_motor2.setVelocity(10)
            elif(cx >340 and cy < 450):
                print("ball on right")
                left_motor1.setVelocity(10)
                left_motor2.setVelocity(10)
                right_motor1.setVelocity(1)
                right_motor2.setVelocity(1)
            elif(cy > 450):
                left_motor1.setVelocity(0)
                left_motor2.setVelocity(0)
                right_motor1.setVelocity(0)
                right_motor2.setVelocity(0)
                if(status == 10):
                    print("Grip")
                    #horizontal_R_motor.setPosition(-0.05)
                    #horizontal_L_motor.setPosition(0.1)
                    robot.step(256)
                    vertical_motor.setPosition(0.05)
                    robot.step(256)
                    
                    horizontal_R_motor.setPosition(0.03)
                    horizontal_L_motor.setPosition(-0.02)
                    vacL.turnOn()
                    vacR.turnOn()
                    robot.step(64*10)
                    vertical_motor.setVelocity(2)
                    vertical_motor.setPosition(0.3)
                    robot.step(64*10)
                    pole_motor.setPosition(3.14)
                    status = 1
                    
                    left_motor1.setVelocity(left_speed)
                    left_motor2.setVelocity(left_speed)
                    right_motor1.setVelocity(0)
                    right_motor2.setVelocity(0)
                    
                    robot.step(64*40)
                    print("pole position set")
                    pole_motor.setPosition(3.14)
                    left_motor1.setVelocity(0)
                    left_motor2.setVelocity(0)
                    right_motor1.setVelocity(0)
                    right_motor2.setVelocity(0)
                
                
            else:
                print("ball on mid")
                left_motor1.setVelocity(left_speed)
                left_motor2.setVelocity(left_speed)
                right_motor1.setVelocity(right_speed)
                right_motor2.setVelocity(right_speed)
        
        
        
        
        if(bumperReading >= 7):
            wall_hitter = wall_hitter+1
            if(wall_hitter >= 20):
                print("Hit the wall")
                left_motor1.setVelocity(-10)
                left_motor2.setVelocity(-10)
                right_motor1.setVelocity(-10)
                right_motor2.setVelocity(-10)
                robot.step(64*10)
        else:
            wall_hitter = 0
            
        i = i+1;
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        pass
    cv2.destroyAllWindows()
    