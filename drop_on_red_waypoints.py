from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
from pymavlink import mavutil
import cv2
import numpy as np
import math
from datetime import datetime
from time import sleep

# def gstreamer_pipeline(
#     capture_width=640,
#     capture_height=360,
#     display_width=640,
#     display_height=360,
#     framerate=30,
#     flip_method=0,
# ):
#     return (
#         "nvarguscamerasrc ! "
#         "video/x-raw(memory:NVMM), "
#         "width=(int)%d, height=(int)%d, "
#         "format=(string)NV12, framerate=(fraction)%d/1 ! "
#         "nvvidconv flip-method=%d ! "
#         "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
#         "videoconvert ! "
#         "video/x-raw, format=(string)BGR ! appsink"
#         % (
#             capture_width,
#             capture_height,
#             framerate,
#             flip_method,
#             display_width,
#             display_height,
#         )
#     )

cap = cv2.VideoCapture(0)
# cap = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)

cap.set(3, 640)# width resolution
cap.set(4, 360)# heigh resolution

vid_cod = cv2.VideoWriter_fourcc(*'mp4v')
filename = "Videos/" + datetime.now().strftime("%Y-%m-%d-%H-%M-%S") + ".mp4"
output = cv2.VideoWriter(filename, vid_cod, 20.0, (640,360))

# Connect to the Vehicle
# vehicle = connect('/dev/ttyUSB0', wait_ready=True, baud=115200)
vehicle = connect('127.0.0.1:14550', wait_ready=True)

cmds = vehicle.commands
cmds.download()
cmds.wait_ready()
vel_x = 0
vel_y = 0
vel_z = 0

pointauth = 2
settime = 0
mode_changed = 0
count = 0
once = False
settime = 0
oncePrinted = False
dropnextwaypoint = False

def send_nav_velocity(velocity_x, velocity_y, velocity_z):
    # create the SET_POSITION_TARGET_LOCAL_NED command
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,  # time_boot_ms (not used)
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
        0b0000111111000111,  # type_mask (only speeds enabled)
        0, 0, 0,  # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z,  # x, y, z velocity in m/s
        0, 0, 0,  # x, y, z acceleration (not used)
        0, 0)  # yaw, yaw_rate (not used)
    # send command to vehicle
    vehicle.send_mavlink(msg)
    vehicle.flush()


def servo(channel, sv):
    # input the message
    msg = vehicle.message_factory.command_long_encode(0, 0,  # target system, target component
                                                      mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                                                      0,  # konfirmasi
                                                      channel,  # pin relay pada AUX OUT 3
                                                      sv,  # pwm value
                                                      0, 0, 0, 0, 0)  # param 1 ~ 5 ga dipake
    # send command to vehicle
    vehicle.send_mavlink(msg)
    vehicle.flush()

def move_target_drop(cX, cY, channel, pwm, waypoint):
    global dropnextwaypoint
    if (dropnextwaypoint == False):
        vehicle.mode = VehicleMode("GUIDED")
        print("Guided: GCS is Controlling")
    vel_x = 0
    vel_y = 0
    vel_z = 0
    if (cY > 270):
        vel_y = -0.125
    elif (cY < 90):
        vel_y = 0.125
    if (cX > 430):
        vel_x = 0.125
    elif (cX < 230):
        vel_x = -0.125
    if (cY < 280 and cY > 100 and cX < 450 and cX > 210):
        send_nav_velocity(0, 0, 0)
        vel_y = 0
        vel_x = 0
        dropnextwaypoint = True
        vehicle.mode = VehicleMode("AUTO")
    send_nav_velocity(vel_y, vel_x, vel_z)

# def move_target_nowaypoiny(cX, cY):
#     vehicle.mode = VehicleMode("GUIDED")
#     vel_x = 0
#     vel_y = 0
#     vel_z = 0
#     if (cY > 270):
#         vel_y = -0.5
#         print("keatas")
#     elif (cY < 90):
#         vel_y = 0.5
#         print("kebawah")
#     if (cX > 430):
#         vel_x = 0.5
#         print("kekiri")
#     elif (cX < 230):
#         vel_x = -0.5
#         print("kekanan")
#     if (cY < 280 and cY > 100 and cX < 450 and cX > 210):
#         send_nav_velocity(0, 0, 0)
#         vel_y = 0
#         vel_x = 0
#         print("ditengah")

#     send_nav_velocity(vel_y, vel_x, vel_z)

def arm_and_takeoff(aTargetAltitude):
    global step
    print("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Tunggu inisiasi...")
        sleep(1)

    print("Arming")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print(" Menunggu arming...")
        sleep(1)

    print("Take off!")
    vehicle.simple_takeoff(1)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        if vehicle.mode == VehicleMode("GUIDED"):
            print(" Altitude: ", vehicle.location.global_relative_frame.alt)
            if aTargetAltitude * 0.95 >= vehicle.location.global_relative_frame.alt >= 1 * 0.95:
                send_nav_velocity(0, 0, -1)
            elif vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
                send_nav_velocity(0, 0, 0)
                print("sampai target altitude")
                step = 1
                break
            sleep(1)
        else:
            break

#arm_and_takeoff(5)

while (True):
    if (vehicle.mode == VehicleMode("STABILIZE")):
        if(oncePrinted == False):
            print("Pilot is controlling")
            oncePrinted = True
    else :
        if (vehicle.mode == VehicleMode("AUTO")):
            print("System waypoints is controlling")
            oncePrinted = False
            nextwaypoint = vehicle.commands.next
        _, img = cap.read()

        # cv2.rectangle(img, (220, 85), (420, 265), (0, 255, 0), 5)
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        orange_lower = np.array([6, 90, 90], np.uint8)
        orange_upper = np.array([19, 255, 255], np.uint8)

        # bluring image
        blur = cv2.GaussianBlur(hsv, (15, 15), cv2.BORDER_DEFAULT)

        orange = cv2.inRange(blur, orange_lower, orange_upper)
        kernal = np.ones((5, 5), "uint8")

        res = cv2.bitwise_and(img, img, mask=orange)

        # Tracking Colour (orange)
        contours, hierarchy = cv2.findContours(orange, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        # contours_roi,hierarchy_roi=cv2.findContours(orange_roi,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

        for pic, contour in enumerate(contours):
            area = cv2.contourArea(contour)
            if (area > 2000):
                c = max(contours, key=cv2.contourArea)
                cv2.drawContours(img, [c], -1, (0, 255, 0), 2)
                M = cv2.moments(c)
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                print (cX, cY)
                cv2.circle(img, (cX, cY), 7, (255, 255, 255), -1)
                if (nextwaypoint == 3):
                      move_target_drop(cX, cY, 5, 1700, nextwaypoint)
                if (nextwaypoint == 4):
                      move_target_drop(cX, cY, 5, 2400, nextwaypoint)
                if (nextwaypoint == 5):
                      move_target_drop(cX, cY, 6, 1700, nextwaypoint)
                # if (nextwaypoint == 6):
                #     move_target_drop(cX, cY, 6, 2400, nextwaypoint)
                # if (nextwaypoint == 7):
                #     move_target_drop(cX, cY, 7, 1700, nextwaypoint)
                # if (nextwaypoint == 8):
                #     move_target_drop(cX, cY, 7, 2400, nextwaypoint)
                # move_target_nowaypoiny(cX, cY,)

        #cv2.imshow("Orange",res)
        # cv2.imshow("Color Tracking",img)
        output.write(img)

        if cv2.waitKey(10) & 0xFF == 27:
            break
        
cap.release()
output.release()
cv2.destroyAllWindows()
