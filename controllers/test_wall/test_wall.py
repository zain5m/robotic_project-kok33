from controller import Robot
from controller.field import Field                             # noqa
from controller.node import Node, ContactPoint                 # noqa
from controller.ansi_codes import AnsiCodes                    # noqa
from controller.accelerometer import Accelerometer             # noqa
from controller.altimeter import Altimeter                     # noqa
from controller.brake import Brake                             # noqa
from controller.camera import Camera, CameraRecognitionObject  # noqa
from controller.compass import Compass                         # noqa
from controller.connector import Connector                     # noqa
from controller.display import Display                         # noqa
from controller.distance_sensor import DistanceSensor          # noqa
from controller.emitter import Emitter                         # noqa
from controller.gps import GPS                                 # noqa
from controller.gyro import Gyro                               # noqa
from controller.inertial_unit import InertialUnit              # noqa
from controller.led import LED                                 # noqa
from controller.lidar import Lidar                             # noqa
from controller.lidar_point import LidarPoint                  # noqa
from controller.light_sensor import LightSensor                # noqa
from controller.motor import Motor                             # noqa
from controller.position_sensor import PositionSensor          # noqa
from controller.radar import Radar                             # noqa
from controller.radar_target import RadarTarget                # noqa
from controller.range_finder import RangeFinder                # noqa
from controller.receiver import Receiver                       # noqa
from controller.robot import Robot                             # noqa
from controller.skin import Skin                               # noqa
from controller.speaker import Speaker                         # noqa
from controller.supervisor import Supervisor                   # noqa
from controller.touch_sensor import TouchSensor                # noqa
from controller.vacuum_gripper import VacuumGripper            # noqa
from controller.keyboard import Keyboard                       # noqa
from controller.mouse import Mouse                             # noqa
from controller.mouse import MouseState                        # noqa
from controller.joystick import Joystick                       # noqa
from controller.motion import Motion                           # noqa
import math
import time

# =========================== {For Color} ===========================
    # HACK:THIS Clean
#    def is_color_detected(self, red, green, blue, target_color):
#         """Helper function to check if a specific color is detected based on RGB values."""
#         if target_color == "red":
#             return red > 200 and red > green + 50 and red > blue + 50
#         elif target_color == "green":
#             return green > 200 and green > red + 50 and green > blue + 50
#         elif target_color == "blue":
#             return blue > 200 and blue > red + 50 and blue > green + 50
#         elif target_color == "yellow":
#             return red > 200 and green > 200 and blue < 100
#         elif target_color == "black":
#             return red < 50 and green < 50 and blue < 50
#         elif target_color == "purple":
#             return red > 150 and blue > 150 and green < 100
#         return False

#     def detect_floor_color(self):
#         """Detect the dominant color on the floor."""
#         image = self.camera_sensor.getImage()
#         width = self.camera_sensor.getWidth()
#         height = self.camera_sensor.getHeight()
        
#         colors_count = {"red": 0, "green": 0, "blue": 0, "yellow": 0, "black": 0, "purple": 0}
#         step = 64

#         for y in range(0, height, step):
#             for x in range(0, width, step):
#                 index = (y * width + x) * 4
#                 red, green, blue = image[index + 2], image[index + 1], image[index]
                
#                 # Increment count for the detected color
#                 for color in colors_count.keys():
#                     if self.is_color_detected(red, green, blue, color):
#                         colors_count[color] += 1
#                         break

#         return max(colors_count, key=colors_count.get) if any(colors_count.values()) else None

#     def track_color(self, target_color):
#         """Track if the specified color is detected."""
#         image = self.camera_sensor.getImage()
#         width = self.camera_sensor.getWidth()
#         height = self.camera_sensor.getHeight()
        
#         step = 64

#         for y in range(0, height, step):
#             for x in range(0, width, step):
#                 index = (y * width + x) * 4
#                 red, green, blue = image[index + 2], image[index + 1], image[index]
                
#                 # Check if the target color is detected
#                 if self.is_color_detected(red, green, blue, target_color):
#                     return True

#         return False


YOUBOT_WHEEL_RADIUS = 0.055 # the raduis of the wheel in E-Puck robot
# YOUBOT_RADIUS = 0.277 # half the distance between E-Puck wheels
# YOUBOT_RADIUS = 0.158 # half the distance between E-Puck wheels

YOUBOT_RADIUS = 0.228
# YOUBOT_RADIUS = 0.456 
YOUBOT_MAX_VELOCITY = 14.81 # youBot motor max velocity

FINGER_MAX_POSITION  = 0.025
FINGER_MIN_POSITION = 0

FINGER_MAX_VELOCITY = 10.0

ARM_MAX_VELOCITY = 1.5708
# 
ARM_1_MAX_POSITION = 2.9496
ARM_1_MIN_POSITION = -2.9496

ARM_2_MAX_POSITION = 1.5708
ARM_2_MIN_POSITION = -1.13446

ARM_3_MAX_POSITION = 2.54818
ARM_3_MIN_POSITION = -2.63545

ARM_4_MAX_POSITION = 1.78024
ARM_4_MIN_POSITION = -1.78024

ARM_5_MAX_POSITION = 2.92343
ARM_5_MIN_POSITION = -2.92343

# RED_COLOR
# RED_COLOR
# RED_COLOR
# RED_COLOR
# red
# green
# blue
# yellow
# purple

# PID Factors
Kp = 0.01
Kd = 0.01
Ki = 0

# Last error to be used by the PID.
last_error = 0

#Integral (the accumulation of errors) to be used by the PID.
integral = 0

def range_conversion(s_start, s_end, d_start, d_end, value):
    """
    This function is responsible for mapping ranges
    examples:
    the mapping of the value 50 from range 0 -> 200 to range -50 -> 50 will be -25
    """
    ration = abs((value - s_start) / (s_end - s_start))
    if(d_start < d_end):
        return  d_start + abs(d_end - d_start) * ration 
    if(d_start > d_end):
        return  d_start - abs(d_end - d_start) * ration 

class RobotController(Robot):
    def __init__(self):
        Robot.__init__(self)
        self.timestep = int(self.getBasicTimeStep())
        
        # Wheel
        self.front_right_wheel = self.getDevice("wheel1")
        self.front_left_wheel = self.getDevice("wheel2")
        self.back_right_wheel = self.getDevice("wheel3")
        self.back_left_wheel = self.getDevice("wheel4")
        
        # Sensor
        self.right_motor_sensor = self.getDevice("wheel1sensor")
        # Sensor enable
        self.right_motor_sensor.enable(self.timestep)
        # Arms
        self.arm1 = self.getDevice("arm1")
        self.arm2 = self.getDevice("arm2")
        self.arm3 = self.getDevice("arm3")
        self.arm4 = self.getDevice("arm4")
        self.arm5 = self.getDevice("arm5")
        self.finger = self.getDevice("finger::left")
        # Arms Sensor
        self.arm1_sensor = self.arm1.getPositionSensor()
        self.arm2_sensor = self.arm2.getPositionSensor()
        self.arm3_sensor = self.arm3.getPositionSensor()
        self.arm4_sensor = self.arm4.getPositionSensor()
        self.arm5_sensor = self.arm5.getPositionSensor()
        self.finger_sensor = self.finger.getPositionSensor()
        # Arms Sensor enable
        self.arm1_sensor.enable(self.timestep)
        self.arm2_sensor.enable(self.timestep)
        self.arm3_sensor.enable(self.timestep)
        self.arm4_sensor.enable(self.timestep)
        self.arm5_sensor.enable(self.timestep)
        self.finger_sensor.enable(self.timestep)
        # Arms Position
        self.arm1.setPosition(0)
        self.arm2.setPosition(0)
        self.arm3.setPosition(0)
        self.arm4.setPosition(0)
        self.arm5.setPosition(0)
        self.finger.setPosition(0)
        # Arms Position Velocity
        self.arm1.setVelocity(ARM_MAX_VELOCITY)
        self.arm2.setVelocity(ARM_MAX_VELOCITY)
        self.arm3.setVelocity(ARM_MAX_VELOCITY)
        self.arm4.setVelocity(ARM_MAX_VELOCITY)
        self.arm5.setVelocity(ARM_MAX_VELOCITY)
        self.finger.setVelocity(FINGER_MAX_VELOCITY)
        # Wheel Position
        self.front_right_wheel.setPosition(float("inf"))
        self.front_left_wheel.setPosition(float("inf"))
        self.back_right_wheel.setPosition(float("inf"))
        self.back_left_wheel.setPosition(float("inf"))
        # Wheel Velocity
        self.front_right_wheel.setVelocity(0)
        self.front_left_wheel.setVelocity(0)
        self.back_right_wheel.setVelocity(0)
        self.back_left_wheel.setVelocity(0)
        self.down = self.getDevice("down")
        self.down1 = self.getDevice("down1")#right 
        self.down2 = self.getDevice("down2")#left
        self.down.enable(self.timestep)
        self.down1.enable(self.timestep)
        self.down2.enable(self.timestep)
        
        # self.compass = self.getDevice("compass")
        # self.compass.enable(self.timestep)
        # self.df = self.getDevice('df1')
        # self.df.enable(self.timestep)
        # Step
        self.step(self.timestep)
        
    def turn_cw(self, velocity):
        self.front_right_wheel.setVelocity(-velocity)
        self.front_left_wheel.setVelocity(velocity)
        self.back_left_wheel.setVelocity(velocity)
        self.back_right_wheel.setVelocity(-velocity)

    def turn_ccw(self, velocity):
        self.front_right_wheel.setVelocity(velocity)
        self.front_left_wheel.setVelocity(-velocity)
        self.back_left_wheel.setVelocity(-velocity)
        self.back_right_wheel.setVelocity(velocity)
    def align_with_wall_using_compass(self,step_size=0.01):
        desired_angle = 180
        angle_tolerance = 0

        while self.step(r.timestep) != -1:
            
            
            compass_values = self.compass.getValues()
            # current_angle = math.atan2(compass_values[0], compass_values[1]) * (180 / math.pi)
            # تنسيق القيم لتكون بثلاثة أرقام إجمالية
            compass_x = float(format(compass_values[0], '.3g'))
            compass_y = float(format(compass_values[1], '.3g'))
            # حساب الزاوية الحالية
            current_angle = math.atan2(compass_x, compass_y) * (180 / math.pi)
        
            if current_angle < 0:
                current_angle += 360
            current_angle = float(format(current_angle, '.3g'))
            
            # print(f"current_angle  => {current_angle}")
            # angle_difference = desired_angle - current_angle
            angle_difference = desired_angle - current_angle
            
            # ضبط الفرق ليكون في النطاق [-180, 180]
            if angle_difference > 180:
                angle_difference -= 360
            elif angle_difference < -180:
                angle_difference += 360
                
            angle_difference = float(format(angle_difference, '.3g'))
            
            print(f"current_angle => {current_angle}")
            print(f"angle_difference => {angle_difference}")

            # إذا كانت الزاوية ضمن النطاق المسموح به، توقف
            if abs(angle_difference) <= angle_tolerance:
                print(f"Aligned with wall. Current angle: {current_angle}")
                self.stop_move()
                break
            elif angle_difference > 0:
                print("Turning counterclockwise...")
                self.turn_ccw(step_size)
            else:
                print("Turning clockwise...")
                step_size = step_size / 2
                self.turn_cw(step_size)
            
    def wait_motors_stopped(self, with_finger=True):
        # "are_motors_stopped"  or "wait_motors_stopped"
        motor_tolerance = 0.01
        motors = [r.arm1,r.arm2,r.arm3,r.arm4,r.arm5]
        sensors = [r.arm1_sensor,r.arm2_sensor,r.arm3_sensor,r.arm4_sensor,r.arm5_sensor]
        finger_tolerance = 0.001
        finger = r.finger
        sensors_finger = r.finger_sensor
        
        while self.step(r.timestep) != -1:
            all_stopped = True
            
            for motor, sensor in zip(motors, sensors):
                current_position = sensor.getValue()
                target_position = motor.getTargetPosition()
                if abs(current_position - target_position) >= motor_tolerance:
                    all_stopped = False 
                    break
            
            if with_finger:
                current_finger_position = sensors_finger.getValue()
                target_finger_position = finger.getTargetPosition()
                if abs(current_finger_position - target_finger_position) >= finger_tolerance:
                    all_stopped = False

            if all_stopped:
                return True
    def set_motors_velocity(self, front_right_v, front_left_v, back_right_v, back_left_v):
        self.front_right_wheel.setVelocity(front_right_v)
        self.front_left_wheel.setVelocity(front_left_v)
        self.back_right_wheel.setVelocity(back_right_v)
        self.back_left_wheel.setVelocity(back_left_v)

    def move_forward(self, velocity = YOUBOT_MAX_VELOCITY):
        self.set_motors_velocity(velocity, velocity, velocity, velocity) 
        
    def move_backward(self, velocity = YOUBOT_MAX_VELOCITY):
        self.set_motors_velocity(-velocity, -velocity, -velocity, -velocity)
        
    def stop_move(self):
        print("stop_move")
        self.set_motors_velocity(0, 0, 0, 0)
    def move_left(self, velocity):
        self.front_right_wheel.setVelocity(velocity)
        self.front_left_wheel.setVelocity(-velocity)
        self.back_left_wheel.setVelocity(velocity)
        self.back_right_wheel.setVelocity(-velocity)

    def move_right(self, velocity):
        self.front_right_wheel.setVelocity(-velocity)
        self.front_left_wheel.setVelocity(velocity)
        self.back_left_wheel.setVelocity(-velocity)
        self.back_right_wheel.setVelocity(velocity)
    def stop_at_wall(self,target_distance=306, tolerance=0, step_size=0):
        min_speed = YOUBOT_MAX_VELOCITY * 0.05
        while self.step(r.timestep) != -1:
            # current_distance = round(self.df.getValue(), 2)  # 2 أرقام بعد الفاصلة
            # target_distance = round(target_distance, 2)     # 2 أرقام بعد الفاصلة
            current_distance = int(self.df.getValue())  # إزالة الأرقام بعد الفاصلة
            target_distance = int(target_distance)     # إزالة الأرقام بعد الفاصلة
            print(f"current_distance > {current_distance}")
            print(f"target_distance > {target_distance}")
            
            if abs(current_distance - target_distance) <= tolerance:
                self.stop_move()
                break
            elif current_distance > target_distance:
                print("move_forward")
                self.move_forward(step_size)
            else:
                print("move_backward")
                if step_size > min_speed:
                    step_size /= 2
                    
                self.move_backward(step_size)
                
    def run_motors_for_rotations_by_right_motor(self,rotation,right_velocity ,left_velocity):
        angle = rotation * 2 * math.pi
        
        curr = self.right_motor_sensor.getValue()
        self.set_motors_velocity(right_velocity,left_velocity,right_velocity,left_velocity)
        
        while((self.right_motor_sensor.getValue() - curr) < angle ):
            self.step(self.timestep)
            
        self.stop_move()

    def move_distance_by_right_motor(
        self,
        distance,
        right_velocity = YOUBOT_MAX_VELOCITY,
        left_velocity = YOUBOT_MAX_VELOCITY
        ):
        rotation = distance / (2 * math.pi * YOUBOT_WHEEL_RADIUS)
        
        self.run_motors_for_rotations_by_right_motor(rotation , right_velocity, left_velocity)
        
r = RobotController()
# while r.step(r.timestep) != -1:
#     vd =r.down.getValue()
#     vdr = r.down1.getValue()# right
#     vdl = r.down2.getValue()# left
#     print(f"vdl ======> {vdl}")
#     print(f"vd ======> {vd}")
#     print(f"vdr ======> {vdr}")
#     if (vdl > 200 and vd > 200 and vdr > 200):
#         # don
#         r.step(r.timestep)
#         r.stop_move()
#         break
#         pass
#     elif (vdl > 200 and vd > 200 and vdr < 200):
#         # Go Left
#         r.move_left(YOUBOT_MAX_VELOCITY / 14)
#         pass        
#     elif (vdl > 200 and vd < 200 and vdr < 200):
#         # Go Left 
#         r.move_left(YOUBOT_MAX_VELOCITY / 14)
#         pass
    
#     elif (vdl < 200 and vd > 200 and vdr > 200):
#         # go Right
#         r.move_right(YOUBOT_MAX_VELOCITY / 14)
#         pass
#     elif (vdl < 200 and vd < 200 and vdr > 200):
#         # go Right
#         r.move_right(YOUBOT_MAX_VELOCITY / 14)
#         pass
#     else :
#         r.move_left(YOUBOT_MAX_VELOCITY / 14)
#         pass
# r.move_forward()
# r.move_distance_by_right_motor(distance=1)
# r.align_with_wall_using_compass(step_size=YOUBOT_MAX_VELOCITY / 3)
# r.stop_at_wall(step_size=YOUBOT_MAX_VELOCITY /3 )
# #  import too zeze
# r.arm2.setPosition(ARM_2_MAX_POSITION / 5)
# r.arm1.setPosition(0)
# r.arm3.setPosition(ARM_3_MAX_POSITION / 2)
# r.arm4.setPosition(ARM_4_MAX_POSITION / 1.5)
# r.arm5.setPosition(0)
# r.finger.setPosition(FINGER_MAX_POSITION)
# r.wait_motors_stopped(True)
# #
# r.arm3.setPosition(ARM_3_MAX_POSITION / 1.8)
# r.wait_motors_stopped()
# # 
# r.finger.setPosition(0)
# r.wait_motors_stopped(True)

# # #TODO: import
# r.arm1.setPosition(0)
# r.arm2.setPosition(0)
# r.arm3.setPosition(ARM_3_MIN_POSITION / 4.3)
# r.arm4.setPosition(ARM_4_MIN_POSITION / 1.9)
# r.arm5.setPosition(0)
# r.wait_motors_stopped()
# r.finger.setPosition(FINGER_MAX_POSITION)
# r.wait_motors_stopped(True)

#TODO: import
# r.align_with_wall_using_compass()

# r.move_forward()
# print("Finish move_forward")
# r.reading_color_matrix()
# print("Finish reading_color_matrix")
# for i in range(4):
#     rank = i + 1
#     rank_color = r.get_color_name(rank)
#     print(f"rank => {rank}")
#     print(f"rank_color => {rank_color}")
#     if(rank_color == "yellow"):
#         r.turn_angle(0.9,clockwise=False)
#         r.move_distance_by_right_motor(0.25)
#         r.PID(rank_color)
#         # Start Fun
#         r.set_motors_velocity(YOUBOT_MAX_VELOCITY,YOUBOT_MAX_VELOCITY,YOUBOT_MAX_VELOCITY,YOUBOT_MAX_VELOCITY)
#         while r.step(r.timestep) != -1:
#             if r.left_sensor.getValue() < 1000:
#                 r.stop_move()
#                 break
#         # Finish Fun
#         pass
#     elif rank_color == "red":
#         r.turn_angle(0.4,clockwise=False)
#         r.move_distance_by_right_motor(0.25)
#         r.PID(rank_color)
#         # Start Fun
#         # TODO: I am Working here babe
#         r.set_motors_velocity(YOUBOT_MAX_VELOCITY,YOUBOT_MAX_VELOCITY,YOUBOT_MAX_VELOCITY,YOUBOT_MAX_VELOCITY)
#         while r.step(r.timestep) != -1:
#             print(f"left_sensor => {r.left_sensor.getValue()}")
#             if r.left_sensor.getValue() < 1000:
#                 r.stop_move()
#                 break
#         print(f"Hold Box")
#         # Hold Box
#         r.arm1.setPosition(ARM_1_MIN_POSITION / 1.8)
#         r.arm2.setPosition(ARM_2_MAX_POSITION)
#         r.arm3.setPosition( ARM_3_MAX_POSITION / 8.3)
#         r.arm4.setPosition(ARM_4_MAX_POSITION / 2.1)
#         r.arm5.setPosition(0)
#         r.finger.setPosition(FINGER_MAX_POSITION)
#         r.wait_motors_stopped(True)
#         r.finger.setPosition(0)
#         r.wait_motors_stopped(True)
        
#         # put on back
#         r.arm2.setPosition(ARM_2_MAX_POSITION / 2)
#         r.wait_motors_stopped()
#         #FIXME: !!!! what this babe !!!!!!!!!! down
#         r.arm2.setPosition(ARM_2_MAX_POSITION / 5)
#         r.arm1.setPosition(0)
#         r.arm3.setPosition(ARM_3_MAX_POSITION / 2)
#         r.arm4.setPosition(ARM_4_MAX_POSITION / 1.5)
#         r.arm5.setPosition(0)
#         r.wait_motors_stopped()
#         #
#         r.arm3.setPosition(ARM_3_MAX_POSITION / 1.8)
#         r.wait_motors_stopped()
#         # 
#         r.finger.setPosition(FINGER_MAX_POSITION)
#         r.wait_motors_stopped(True)
#         r.set_motors_velocity(YOUBOT_MAX_VELOCITY,YOUBOT_MAX_VELOCITY,YOUBOT_MAX_VELOCITY,YOUBOT_MAX_VELOCITY)
#         while r.step(r.timestep) != -1:
#             if r.track_color("black"):
#                 break            
#         r.PID("purple")
#         print("finished PId and this new Fun")
#         break
#         #TODO: I am going to test this 
        
#         # Finish Fun
#         pass
#     elif rank_color == "green":
#         r.turn_angle(0.4,clockwise=True)
#         r.move_distance_by_right_motor(0.25)
#         r.PID(rank_color)
#         # Start Fun
#         # TODO: I am Working here babe
#         r.set_motors_velocity(YOUBOT_MAX_VELOCITY,YOUBOT_MAX_VELOCITY,YOUBOT_MAX_VELOCITY,YOUBOT_MAX_VELOCITY)
#         while r.step(r.timestep) != -1:
#             print(f"left_sensor => {r.left_sensor.getValue()}")
#             if r.left_sensor.getValue() < 1000:
#                 r.stop_move()
#                 break
#         print(f"Hold Box")
#         # Hold Box
#         r.arm1.setPosition(ARM_1_MIN_POSITION / 1.8)
#         r.arm2.setPosition(ARM_2_MAX_POSITION)
#         r.arm3.setPosition( ARM_3_MAX_POSITION / 8.3)
#         r.arm4.setPosition(ARM_4_MAX_POSITION / 2.1)
#         r.arm5.setPosition(0)
#         r.finger.setPosition(FINGER_MAX_POSITION)
#         r.wait_motors_stopped(True)
#         r.finger.setPosition(0)
#         r.wait_motors_stopped(True)
        
#         # put on back (ارفع اليد قليلا)
#         r.arm2.setPosition(ARM_2_MAX_POSITION / 2)
#         r.wait_motors_stopped()
#         #FIXME: !!!! what this babe !!!!!!!!!! down
#         #   وضع اليد على الظهر
#         r.arm2.setPosition(ARM_2_MAX_POSITION / 5)
#         r.arm1.setPosition(0)
#         r.arm3.setPosition(ARM_3_MAX_POSITION / 2)
#         r.arm4.setPosition(ARM_4_MAX_POSITION / 1.5)
#         r.arm5.setPosition(0)
#         r.wait_motors_stopped()
#         #   انزل اليد للاخير 
#         r.arm3.setPosition(ARM_3_MAX_POSITION / 1.8)
#         r.wait_motors_stopped()
#         # ترك الكعب
#         r.finger.setPosition(FINGER_MAX_POSITION)
#         r.wait_motors_stopped(True)
#         r.set_motors_velocity(YOUBOT_MAX_VELOCITY,YOUBOT_MAX_VELOCITY,YOUBOT_MAX_VELOCITY,YOUBOT_MAX_VELOCITY)
#         while r.step(r.timestep) != -1:
#             if r.track_color("black"):
#                 break            
#         r.PID("purple")
#         print("finished PId and this new Fun")
#         break
#         #TODO: I am going to test this 
        
#         # Finish Fun
#         pass
#     elif rank_color == "blue":
#         r.turn_angle(0.9,clockwise=True)
#         r.move_distance_by_right_motor(0.25)
#         r.PID(rank_color)
#         pass