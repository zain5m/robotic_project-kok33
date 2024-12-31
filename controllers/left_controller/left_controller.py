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
from colorama import Fore, Back, Style

class StyledPrinter:
    def __init__(self, prefix_text="LEFT ROBOT =>", prefix_style=Back.GREEN + Style.BRIGHT):
        self.prefix_text = prefix_text
        self.prefix_style = prefix_style
        self.reset_style = Style.RESET_ALL

    def print(self, text):
        print(f"{self.prefix_style}{self.prefix_text}{self.reset_style} {text}")

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

RED_COLOR = "red"
GREEN_COLOR = "green"
BLUE_COLOR = "blue"
YELLOW_COLOR = "yellow"    
PURPLE_COLOR = "purple"
BLACK_COLOR = "black"

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
        
        self.printer = StyledPrinter()

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
        # Sensors For Line
        self.sensors = list(map(lambda v: self.getDevice(f"lfs{v}"), range(8)))
        # Weights For Sensors Line
        self.weights = [-1000, -1000, -1000, -1000, 1000, 1000, 1000, 1000]
        # Sensors For Line Enable
        for sensor in self.sensors:
            sensor.enable(self.timestep)
        # Camera 
        self.camera_sensor = self.getDevice("camera")
        self.camera_sensor.enable(self.timestep)
        
        # BOX Sensor
        self.left_sensor = self.getDevice("left sensor")
        self.left_sensor.enable(self.timestep)
        
        # Compass
        self.compass = self.getDevice("compass")
        self.compass.enable(self.timestep)
        
        # Front Sensor For Wall
        self.front_sensor = self.getDevice('front_sensor')
        self.front_sensor.enable(self.timestep)
        # 
        self.down_right = self.getDevice("down_right")
        self.down_center = self.getDevice("down_center")
        self.down_left = self.getDevice("down_left")
        self.down_right.enable(self.timestep)
        self.down_center.enable(self.timestep)
        self.down_left.enable(self.timestep)
        # Hand sensor
        self.hand_sensor = self.getDevice("hand_sensor")
        self.hand_sensor.enable(self.timestep)
        # Variable For Me
        self.isColorMatrixReadingFinished = False
        self.arrayColor = {"red": 0, "green": 0, "blue": 0, "yellow": 0}
        
        # Step
        self.step(self.timestep)
    
    # =========================== For Color ===========================
    def __detect_floor_color(self):
        image = self.camera_sensor.getImage()
        width = self.camera_sensor.getWidth()
        height = self.camera_sensor.getHeight()
        
        colors_count = {RED_COLOR: 0, GREEN_COLOR: 0, BLUE_COLOR: 0, YELLOW_COLOR: 0, PURPLE_COLOR: 0 }
        
        step = 64
        for y in range(0,height,step):
            for x in range(0, width,step):
                
                index = (y * width + x) * 4
                red, green, blue = image[index + 2], image[index + 1], image[index]
                
                if red > 200 and red > green + 50 and red > blue + 50:
                    colors_count[RED_COLOR] += 1
                elif green > 200 and green > red + 50 and green > blue + 50:
                    colors_count[GREEN_COLOR] += 1
                elif blue > 200 and blue > red + 50 and blue > green + 50:
                    colors_count[BLUE_COLOR] += 1
                elif red > 200 and green > 200 and blue < 100:
                    colors_count[YELLOW_COLOR] += 1
                # elif red < 50 and green < 50 and blue < 50:
                #     colors_count["black"] += 1
                # elif red > 120 and blue > 120 and green < 50:  
                elif red > 150 and blue > 150 and green < 100:
                    colors_count[PURPLE_COLOR] += 1

        return max(colors_count, key=colors_count.get) if any(colors_count.values()) else None
    
    def track_color(self, target_color):
        image = self.camera_sensor.getImage()
        width = self.camera_sensor.getWidth()
        height = self.camera_sensor.getHeight()
        step = 64
        
        for y in range(0, height, step):
            for x in range(0, width, step):
                index = (y * width + x) * 4
                red, green, blue = image[index + 2], image[index + 1], image[index]
                
                if target_color == RED_COLOR and red > 200 and red > green + 50 and red > blue + 50:
                    return True
                elif target_color == GREEN_COLOR and green > 200 and green > red + 50 and green > blue + 50:
                    return True
                elif target_color == BLUE_COLOR and blue > 200 and blue > red + 50 and blue > green + 50:
                    return True
                elif target_color == YELLOW_COLOR and red > 200 and green > 200 and blue < 100:
                    return True
                elif target_color == BLACK_COLOR and red < 50 and green < 50 and blue < 50:
                    return True
                elif target_color == PURPLE_COLOR and red > 150 and blue > 150 and green < 100:
                    return True

        return False

    def reading_color_matrix(self):
        while r.step(r.timestep) != -1:
            if r.isColorMatrixReadingFinished:
                break
            floor_color = self.__detect_floor_color()
            if(floor_color != None):
                if(floor_color == PURPLE_COLOR):
                    self.isColorMatrixReadingFinished = True
                    self.printer.print("i reading purple || Finished reading_color_matrix")
                    self.printer.print(f"arrayColor  =>  {self.arrayColor}")
                elif(self.arrayColor[floor_color] == 0):
                    ranking = max(self.arrayColor.values())
                    self.arrayColor[floor_color] = ranking + 1

    def get_color_name(self, rank):
        my_color = None
        for color, value in self.arrayColor.items():
            if value == rank:
                return color
                # my_color = color
        return "Color not found" 

    # =========================== Passive Wait ===========================
    def passive_wait(self, sec):
        start_time = self.getTime()
        # print("start_time", start_time)
        while start_time + sec > self.getTime():
            # print("########################################")
            # print(self.getTime(),sec,start_time,start_time+sec)
            self.step()

    # =========================== PID Line Following ===========================    
    def get_sensors_value(self):
        value = 0
        for index, sensor in enumerate(self.sensors):
            if(sensor.getValue() > 300):
                value += self.weights[index]

        return value

    def PID_step(self, velocity = YOUBOT_MAX_VELOCITY):
        global last_error, integral
        value = self.get_sensors_value()
        # print(value)
        error = 0 - value
        # Get P term of the PID.
        P = Kp * error
        # Get D term of the PID.
        D = Kd * (last_error - error)
        # Update last_error to be used in the next iteration.
        last_error = error
        # Get I term of the PID.
        I = Ki * integral
        # Update intergral to be used in the next iteration.
        integral += error

        steering = P + D + I
        
        # left_velocity = velocity + steering
        
        # right_velocity = velocity - steering
        # ////
        # left_velocity = min(YOUBOT_MAX_VELOCITY, max(-YOUBOT_MAX_VELOCITY, velocity + steering))
        # right_velocity = min(YOUBOT_MAX_VELOCITY, max(-YOUBOT_MAX_VELOCITY, velocity - steering))

        # self.front_left_wheel.setVelocity(left_velocity)
        # self.back_left_wheel.setVelocity(left_velocity)
        # self.front_right_wheel.setVelocity(right_velocity)
        # self.back_right_wheel.setVelocity(right_velocity)
        # self.run_motors_stearing(steering,velocity)
        self.run_motors_stearing(steering,velocity)
        
    def run_motors_stearing(self, stearing, velocity = YOUBOT_MAX_VELOCITY):
        """
        A function that is responsible for the steering functionality for the motor
        Steering value:
            - from -100 to 0 will turn left
            - from 0 to 100 will turn right
            - if equals 100 will turn the robot around it self to the right
            - if equals -100 will turn the robot around it self to the left
            - if equals 50 will turn the robot around right wheel to the right
            - if equals -50 will turn the robot around left wheel to the left
        """
        right_velocity = velocity if stearing < 0 else range_conversion(0, 100, velocity, -velocity, stearing)
        left_velocity = velocity if stearing > 0 else range_conversion(0, -100, velocity, -velocity, stearing)
        self.front_left_wheel.setVelocity(left_velocity)
        self.back_left_wheel.setVelocity(left_velocity)
        self.front_right_wheel.setVelocity(right_velocity)
        self.back_right_wheel.setVelocity(right_velocity)
        
    def PID(self, rank_color,velocity = YOUBOT_MAX_VELOCITY):
        self.printer.print(f"Start PID & Search Color {rank_color}")
        while r.step(r.timestep) != -1:
            if self.track_color(rank_color):
                self.printer.print(f"Finished PID & Find Color {rank_color}")
                break
            self.PID_step(velocity)

    # =========================== Wall ===========================    

    def stop_at_wall(self, target_distance=306, velocity=YOUBOT_MAX_VELOCITY / 3):
        tolerance = 0
        min_speed = YOUBOT_MAX_VELOCITY * 0.05
        while self.step(r.timestep) != -1:
            current_distance = int(self.front_sensor.getValue()) 
            target_distance = int(target_distance)
            # print(f"current_distance > {current_distance}")
            # print(f"target_distance > {target_distance}")
            if abs(current_distance - target_distance) <= tolerance:
                self.stop_move()
                break
            elif current_distance > target_distance:
                # print("move_forward")
                self.move_forward(velocity)
            else:
                # print("move_backward")
                if velocity > min_speed:
                    velocity /= 2
                self.move_backward(velocity)
    
    def center_robot_at_wall(self,velocity = YOUBOT_MAX_VELOCITY / 14,right = False):
        if right:
            self.move_right(velocity)
        else:
            self.move_left(velocity)
        vv = 200
        while self.step(self.timestep) != -1:
            value_down_right = float(format(self.down_right.getValue(), '.3g'))
            value_down_center = float(format(self.down_center.getValue(), '.3g'))
            value_down_left = float(format(self.down_left.getValue(), '.3g'))
            self.printer.print(f"right  = > {value_down_right}")
            self.printer.print(f"center  = > {value_down_center}")
            self.printer.print(f"left  = > {value_down_left}")
            if (value_down_left > vv and value_down_center > vv and value_down_right > vv):
                # don
                self.printer.print(f"Center At Wall R={value_down_right} C={value_down_center} L={value_down_left}")
                self.step(self.timestep)
                self.stop_move()
                break
            elif (value_down_left > vv and value_down_center > vv and value_down_right < vv):
                # Go Left
                # print("go Left  Right <")
                self.move_left(velocity)
                pass        
            elif (value_down_left > vv and value_down_center < vv and value_down_right < vv):
                # Go Left 
                # print("go Left  Center <  || Right <")
                self.move_left(velocity)
                pass
            
            elif (value_down_left < vv and value_down_center > vv and value_down_right > vv):
                # go Right
                # print("go Right Left<")
                self.move_right(velocity)
                pass
            elif (value_down_left < vv and value_down_center < vv and value_down_right > vv):
                # go Right
                # print("go Right Left< || center <")
                self.move_right(velocity)
                pass
            else :
                pass
                # print("else")
                
            #     pass

    def r_center_robot_at_wall(self,velocity = YOUBOT_MAX_VELOCITY / 14,right = False):
        if right:
            self.move_right(velocity)
        else:
            self.move_left(velocity)
        vv = 500
        while self.step(self.timestep) != -1:
            # value_down_right = self.down_right.getValue()
            # value_down_center =self.down_center.getValue()
            # value_down_left = self.down_left.getValue()
            # self.printer.print(f"right  = > {value_down_right}")
            # self.printer.print(f"center  = > {value_down_center}")
            # self.printer.print(f"left  = > {value_down_left}")
            value_down_right = float(format(self.down_right.getValue(), '.3g'))
            value_down_center = float(format(self.down_center.getValue(), '.3g'))
            value_down_left = float(format(self.down_left.getValue(), '.3g'))
            self.printer.print(f"right  = > {value_down_right}")
            self.printer.print(f"center  = > {value_down_center}")
            self.printer.print(f"left  = > {value_down_left}")
            if (value_down_left < vv and value_down_center < vv and value_down_right < vv):
                # don
                self.printer.print(f"Center At Wall R={value_down_right} C={value_down_center} L={value_down_left}")
                self.step(self.timestep)
                self.stop_move()
                break
            elif (value_down_left < vv and value_down_center < vv and value_down_right > vv):
                # Go Left
                # print("go Left  Right <")
                self.move_left(velocity)
                pass        
            elif (value_down_left < vv and value_down_center > vv and value_down_right > vv):
                # Go Left 
                # print("go Left  Center <  || Right <")
                self.move_left(velocity)
                pass
            
            elif (value_down_left > vv and value_down_center < vv and value_down_right < vv):
                # go Right
                # print("go Right Left<")
                self.move_right(velocity)
                pass
            elif (value_down_left > vv and value_down_center > vv and value_down_right < vv):
                # go Right
                # print("go Right Left< || center <")
                self.move_right(velocity)
                pass
            else :
                pass
                # print("else")
                
    # =========================== Move ===========================
    def set_motors_velocity(self, front_right_v, front_left_v, back_right_v, back_left_v):
        self.front_right_wheel.setVelocity(front_right_v)
        self.front_left_wheel.setVelocity(front_left_v)
        self.back_right_wheel.setVelocity(back_right_v)
        self.back_left_wheel.setVelocity(back_left_v)

    def move_forward(self, velocity = YOUBOT_MAX_VELOCITY):
        self.set_motors_velocity(velocity, velocity, velocity, velocity)
        
    def move_backward(self, velocity = YOUBOT_MAX_VELOCITY):
        self.set_motors_velocity(-velocity, -velocity, -velocity, -velocity)

    def move_left(self, velocity = YOUBOT_MAX_VELOCITY):
        self.front_right_wheel.setVelocity(velocity)
        self.front_left_wheel.setVelocity(-velocity)
        self.back_left_wheel.setVelocity(velocity)
        self.back_right_wheel.setVelocity(-velocity)

    def move_right(self, velocity = YOUBOT_MAX_VELOCITY):
        self.front_right_wheel.setVelocity(-velocity)
        self.front_left_wheel.setVelocity(velocity)
        self.back_left_wheel.setVelocity(-velocity)
        self.back_right_wheel.setVelocity(velocity)

    def turn_cw(self, velocity = YOUBOT_MAX_VELOCITY):
        self.front_right_wheel.setVelocity(-velocity)
        self.front_left_wheel.setVelocity(velocity)
        self.back_left_wheel.setVelocity(velocity)
        self.back_right_wheel.setVelocity(-velocity)

    def turn_ccw(self, velocity = YOUBOT_MAX_VELOCITY):
        self.front_right_wheel.setVelocity(velocity)
        self.front_left_wheel.setVelocity(-velocity)
        self.back_left_wheel.setVelocity(-velocity)
        self.back_right_wheel.setVelocity(velocity)
    
    def stop_move(self):
        # print("stop_move")
        self.set_motors_velocity(0, 0, 0, 0)
    
    def run_motors_for_rotations_by_right_motor(self,rotation,right_velocity ,left_velocity,back =False):
        angle = rotation * 2 * math.pi
        curr = self.right_motor_sensor.getValue()
        if back:
            self.set_motors_velocity(-right_velocity, -left_velocity, -right_velocity, -left_velocity)
            target_angle = curr - angle 
            while self.right_motor_sensor.getValue() > target_angle:
                self.step(self.timestep)
        else:
            self.set_motors_velocity(right_velocity, left_velocity, right_velocity, left_velocity)
            target_angle = curr + angle
            while self.right_motor_sensor.getValue() < target_angle:
                self.step(self.timestep)
        
        self.stop_move()

    def move_distance_by_right_motor(self, distance, right_velocity = YOUBOT_MAX_VELOCITY, left_velocity = YOUBOT_MAX_VELOCITY,back =False):
        rotation = distance / (2 * math.pi * YOUBOT_WHEEL_RADIUS)
        
        self.run_motors_for_rotations_by_right_motor(abs(rotation) , right_velocity, left_velocity,back )

    def turn_angle(self, angle, velocity=YOUBOT_MAX_VELOCITY / 3 ,clockwise = False):
        angle_tolerance = 0
        min_speed = YOUBOT_MAX_VELOCITY * 0.05
        while self.step(r.timestep) != -1:
            compass_values = self.compass.getValues()
            compass_x = float(format(compass_values[0], '.3g'))
            compass_y = float(format(compass_values[1], '.3g'))
            
            current_angle = math.atan2(compass_x, compass_y) * (180 / math.pi)
            
            if current_angle < 0:
                current_angle += 360
            current_angle = float(format(current_angle, '.3g'))
            
            angle_difference = angle - current_angle
            
            if angle_difference > 180:
                angle_difference -= 360
            elif angle_difference < -180:
                angle_difference += 360
            
            angle_difference = float(format(angle_difference, '.3g'))
            
            # print(f"angle_difference  = > {angle_difference}")
            # print(f"current_angle = > {current_angle}")
            if abs(angle_difference) <= angle_tolerance:
                self.printer.print(f"Aligned with wall. Current angle: {current_angle}")
                self.stop_move()
                break
            elif angle_difference > 0:
                if(clockwise):
                    if velocity > min_speed:
                        velocity /= 2
                # print("Turning counterclockwise...")
                self.turn_ccw(velocity)
            else:
                # print("Turning clockwise...")
                if(not clockwise):
                    if velocity > min_speed:
                        velocity /= 2
                self.turn_cw(velocity)
                
            # if abs(angle_difference) < 30 and velocity > min_speed:
            #     velocity /= 2
    # =========================== new ===========================
    
    # =========================== Hand ===========================

    def wait_motors_stopped(self, motor_indices = None,with_finger=False):
        # "are_motors_stopped"  or "wait_motors_stopped"
        motor_tolerance = 0.001
        motors = [r.arm1,r.arm2,r.arm3,r.arm4,r.arm5]
        sensors = [r.arm1_sensor,r.arm2_sensor,r.arm3_sensor,r.arm4_sensor,r.arm5_sensor]
        finger_tolerance = 0.001
        finger = r.finger
        sensors_finger = r.finger_sensor
        
        
        if motor_indices is None:
            motor_indices = range(len(motors))

        stable_steps_threshold = 10
        stopped_positions = {} 
        previous_positions = {} 
        stable_steps = 0
        stable_steps_finger = 0
        for i in motor_indices:
            previous_positions[motors[i - 1].getName()] = 0
        if with_finger:
            previous_positions["finger"] = 0
        while self.step(r.timestep) != -1:
            all_stopped = True
            for i in motor_indices:
                motor = motors[i - 1]
                sensor = sensors[i - 1]
                # current_position = sensor.getValue()
                # target_position = motor.getTargetPosition()
                current_position = float(format(sensor.getValue(), '.3g'))
                target_position = float(format(motor.getTargetPosition(), '.3g'))
                # self.printer.print(f"{motor.getName()}  =>>>  {current_position} - {target_position} = {abs(current_position - target_position)}")
                if abs(current_position - target_position) >= motor_tolerance:
                        
                    if stable_steps >= stable_steps_threshold:
                        pass
                    else:
                        if previous_positions[motor.getName()] == current_position:
                            stable_steps += 1
                        else:
                            previous_positions[motor.getName()] = current_position
                            stable_steps = 0
                        all_stopped = False
                        break
                else:
                    # self.printer.print(f"{motor.getName()}  =>>>  {current_position} - {target_position} = {abs(current_position - target_position)}")
                    stopped_positions[motor.getName()] = abs(current_position - target_position)
                
            if with_finger:
                # current_finger_position = sensors_finger.getValue()
                # target_finger_position = finger.getTargetPosition()
                current_finger_position = float(format(sensors_finger.getValue(), '.3g'))
                target_finger_position = float(format(finger.getTargetPosition(), '.3g'))
                # self.printer.print(f"finger  =>>>  {current_finger_position} - {target_finger_position} = {abs(current_finger_position - target_finger_position)}")
                if abs(current_finger_position - target_finger_position) >= finger_tolerance:
                    # if stable_steps_finger >= stable_steps_threshold:
                    #     pass
                    # else:
                    #     if previous_positions['finger'] == current_finger_position:
                    #         stable_steps_finger += 1
                    #     else:
                    #         previous_positions['finger'] = current_finger_position
                    #         stable_steps_finger = 0
                    all_stopped = False
                else:
                    # self.printer.print(f"finger  =>>>  {current_finger_position} - {target_finger_position} = {abs(current_finger_position - target_finger_position)}")
                    stopped_positions["finger"] = abs(current_finger_position - target_finger_position) 
                    
            if all_stopped:
                self.printer.print("Motors stopped at positions:")
                for motor_name, position in stopped_positions.items():
                    self.printer.print(f"{motor_name}: {position}")
                return True

    def find_box(self):
        # 9.976854691969093 -5.015840463260189 0.10193880936125882
        # 0.0004201827465709361 -1.5326346569649164e-05 0.9999999116057775 -3.1222160845024174
        r.move_forward()
        while self.step(self.timestep) != -1:
            # print(f"left_sensor => {self.left_sensor.getValue()}")
            if self.left_sensor.getValue() < 1000:
                self.stop_move()
                break

    def put_on_ground(self):
        self.arm2.setPosition(-0.62)
        self.arm3.setPosition(-0.98)
        self.arm4.setPosition(-1.53)
        self.arm5.setPosition(0.0)
        self.arm1.setPosition(math.pi / 2)
        self.wait_motors_stopped([2,3,4,5,1])
        
        
        self.arm2.setPosition(-0.97)
        self.arm3.setPosition(-1.55)
        self.arm4.setPosition(-0.61)
        self.arm5.setPosition(0.0)
        self.wait_motors_stopped([2,3,4,5])
        
        self.finger.setPosition(FINGER_MAX_POSITION)
        self.wait_motors_stopped([],True)
        
    def hold_box_from_ground(self):
        self.arm2.setPosition(-0.62)
        self.arm3.setPosition(-0.98)
        self.arm4.setPosition(-1.53)
        self.arm5.setPosition(0.0)
        self.arm1.setPosition(math.pi / 2)
        self.wait_motors_stopped([2,3,4,5,1])
        self.finger.setPosition(FINGER_MAX_POSITION)
        self.wait_motors_stopped([],True)
        self.arm2.setPosition(-0.97)
        self.arm3.setPosition(-1.55)
        self.arm4.setPosition(-0.61)
        self.arm5.setPosition(0.0)
        self.wait_motors_stopped([2,3,4,5])
        self.finger.setPosition(0)
        self.wait_motors_stopped([],True)
        # self.arm1.setPosition(ARM_1_MIN_POSITION / 1.8)
        # self.arm2.setPosition(ARM_2_MAX_POSITION)
        # self.arm3.setPosition( ARM_3_MAX_POSITION / 8.3)
        # self.arm4.setPosition(ARM_4_MAX_POSITION / 2)
        # self.arm5.setPosition(0)
        # self.finger.setPosition(FINGER_MAX_POSITION)
        # self.wait_motors_stopped(True)
        # self.finger.setPosition(0)
        # self.wait_motors_stopped(True)
   
    def reset_arm(self):
        self.arm2.setPosition(0)
        self.arm3.setPosition(0)
        self.arm4.setPosition(0)
        self.wait_motors_stopped([2,3,4])
        
        self.arm1.setPosition(0)
        self.wait_motors_stopped([1])
        self.finger.setPosition(0)
        self.wait_motors_stopped([],True)
        
        
    def put_box_on_back(self, n):
        if n ==1:
            self.arm3.setPosition(0.42)
            self.arm4.setPosition(1.78)
            self.wait_motors_stopped([3,4])
            # self.arm2.setPosition(0.92)
            self.arm2.setPosition(0.90)
            self.wait_motors_stopped([2])    
            # self.finger.setPosition(FINGER_MAX_POSITION)
            # self.wait_motors_stopped([],True)
        if n == 4 or n == 5:
            if n == 4: self.arm1.setPosition(ARM_1_MAX_POSITION)
            if n == 5: self.arm1.setPosition(ARM_1_MIN_POSITION)

            self.arm3.setPosition(- 0.42)
            self.arm4.setPosition(- 1.78)
            self.wait_motors_stopped([3,4])
            # self.arm2.setPosition(0.92)
            self.arm2.setPosition(-0.90)
            self.wait_motors_stopped([2])    
            # self.finger.setPosition(FINGER_MAX_POSITION)
            # self.wait_motors_stopped([],True)
            
        
        if n == 2 or n == 3 :
            if n == 2: self.arm1.setPosition(0.3)
            if n == 3 : self.arm1.setPosition(-0.3)
            self.arm3.setPosition(0.42)
            self.arm4.setPosition(1.78)
            self.wait_motors_stopped([3,4])
            # self.arm2.setPosition(0.92)
            self.arm2.setPosition(0.90)
            self.wait_motors_stopped([2])   
            # self.finger.setPosition(FINGER_MAX_POSITION)
            # self.wait_motors_stopped([],True)
        if n != 6:
            self.finger.setPosition(FINGER_MAX_POSITION)
            self.wait_motors_stopped([],True)

    def v(self,n):
        if n != 6:
            self.finger.setPosition(FINGER_MAX_POSITION)
            self.wait_motors_stopped([],True)

        if n == 1:
            self.arm3.setPosition(0.42)
            self.arm4.setPosition(1.78)
            self.wait_motors_stopped([3,4])
            # self.arm2.setPosition(0.92)
            self.arm2.setPosition(0.90)
            self.wait_motors_stopped([2])
        
        
        
        if n == 4 or n == 5:
            if n == 4: self.arm1.setPosition(ARM_1_MAX_POSITION)
            if n == 5: self.arm1.setPosition(ARM_1_MIN_POSITION)
            self.arm3.setPosition(- 0.42)
            self.arm4.setPosition(- 1.78)
            self.wait_motors_stopped([3,4])
            self.arm2.setPosition(-0.92)
            # self.arm2.setPosition(-0.90)
            self.wait_motors_stopped([2])    
            # self.finger.setPosition(FINGER_MAX_POSITION)
            # self.wait_motors_stopped([],True)
            
        if n == 2 or n == 3 :
            if n == 2: self.arm1.setPosition(0.3)
            if n == 3 : self.arm1.setPosition(-0.3)
            self.arm3.setPosition(0.42)
            self.arm4.setPosition(1.78)
            self.wait_motors_stopped([3,4])
            self.arm2.setPosition(0.92)
            # self.arm2.setPosition(0.90)
            self.wait_motors_stopped([2])    
            # self.finger.setPosition(FINGER_MAX_POSITION)
            # self.wait_motors_stopped([],True)
        if n != 6:
            self.finger.setPosition(0)
            self.wait_motors_stopped([],True)
            # self.arm3.setPosition(0.42)
            # self.arm4.setPosition(1.78)
            # self.wait_motors_stopped([3,4])
            # self.arm2.setPosition(0.92)
            # # self.arm2.setPosition(0.90)
            # self.wait_motors_stopped([2])    
        
        # 
        

  
     # def get_from_back(self):
    #     self.finger.setPosition(FINGER_MAX_POSITION)
    #     self.wait_motors_stopped([],True)
        
    #     self.arm3.setPosition(0.42)
    #     self.arm4.setPosition(1.78)
    #     self.wait_motors_stopped([3,4])
        
    #     self.arm2.setPosition(0.92)
    #     self.wait_motors_stopped([2])
    #     self.finger.setPosition(0)
    #     self.wait_motors_stopped([],True)
        
    #     # self.arm2.setPosition(ARM_2_MAX_POSITION / 5)
    #     # self.arm1.setPosition(0)
    #     # self.arm3.setPosition(ARM_3_MAX_POSITION / 2)
    #     # self.arm4.setPosition(ARM_4_MAX_POSITION / 1.5)
    #     # self.arm5.setPosition(0)
    #     # self.finger.setPosition(FINGER_MAX_POSITION)
    #     # self.wait_motors_stopped(True)
    #     # #
    #     # self.arm3.setPosition(ARM_3_MAX_POSITION / 1.8)
    #     # self.wait_motors_stopped()
    #     # # 
    #     # self.finger.setPosition(0)
    #     # self.wait_motors_stopped(True)
    
    def put_on_wall(self):
        self.arm1.setPosition(0)
        self.arm2.setPosition(0)
        self.arm3.setPosition(ARM_3_MIN_POSITION / 4.2)
        self.arm4.setPosition(ARM_4_MIN_POSITION / 1.9)
        self.arm5.setPosition(0)
        self.wait_motors_stopped([1,2,3,4,5])
        self.finger.setPosition(FINGER_MAX_POSITION)
        self.wait_motors_stopped([],True)

    def get_from_wall(self):
        self.finger.setPosition(FINGER_MAX_POSITION)
        # self.printer.print('wait_motors_stopped')
        self.wait_motors_stopped([],True)
        # self.printer.print('wait_motors_stopped')
        self.arm1.setPosition(0)
        self.arm2.setPosition(0)
        self.arm3.setPosition(ARM_3_MIN_POSITION / 4.2)
        self.arm4.setPosition(ARM_4_MIN_POSITION / 1.9)
        self.arm5.setPosition(0)
        # self.printer.print('wait_motors_stopped 1 2 3 4')
        self.wait_motors_stopped([1,2,3,4,5])
        # self.printer.print('wait_motors_stopped 1 2 3 4')
        self.finger.setPosition(0)
        # self.printer.print('wait_motors_stopped F')
        self.wait_motors_stopped([],True)
        # self.printer.print('wait_motors_stopped F')
        
    def is_box_in_wall(self) -> bool:
        while self.step(r.timestep)  !=-1:
            value_hand_sensor = self.hand_sensor.getValue()
            if value_hand_sensor < 1000:
                return True

    # =========================== Final ===========================
    def go_yellow_way(self):
        self.move_distance_by_right_motor(0.7)
        self.turn_angle(270)
        self.move_distance_by_right_motor(0.3)
        pass
    
    def go_red_way(self):
        self.move_distance_by_right_motor(0.9)
        self.turn_angle(230)
        self.move_distance_by_right_motor(0.5)
        pass
    
    def go_green_way(self):
        self.move_distance_by_right_motor(0.9)
        self.turn_angle(130 ,clockwise=True)
        self.move_distance_by_right_motor(0.5)
        pass
    
    def go_blue_way(self):
        self.move_distance_by_right_motor(0.7)
        self.turn_angle(90, clockwise = True)
        self.move_distance_by_right_motor(0.3)
        pass

    def final(self, rank_color):
        self.PID(rank_color)
        self.printer.print("Turn Angle 180 Before Colors Region")
        self.turn_angle(180)
        # NOTE: ROBOT on RED True
        # 11.257589227488062 -5.010093418538711 0.10193771737672813
        # -0.001054977917274537 -2.1266479059532775e-05 0.9999994432845106 -3.139620641980832
        # NOTE: ROBOT on YELLOW True
        # 11.263043471202858 -11.013264296385241 0.10193771738469964 
        # -0.0010549023627892003 -2.518927640202035e-05 0.9999994432730978 -3.1322164756784736
        # NOTE: ROBOT on GREEN True
        # 11.25794352935697 4.977799316485834 0.10193771736232032
        # -0.0010549892740478117 -2.0661540708924318e-05 0.9999994432852113 -3.140803428902099
        # # NOTE: ROBOT on BLUE True
        # 11.257820522674624 10.986158257040426 0.10193771737941176
        # -0.0010549648577149115 -2.194747370791696e-05 0.9999994432835737 -3.1383873406805765
        # TEST:
        # return
        # TODO:
        for i in range(6):
            n = 6 - i
            self.move_distance_by_right_motor(distance=0.11,right_velocity= YOUBOT_MAX_VELOCITY / 2, left_velocity= YOUBOT_MAX_VELOCITY / 2)
            # Put this in  find_box and rename => find_box_hold
            self.printer.print("Start put BOX")
            self.v(n)
            self.reset_arm()
            self.put_on_ground()
            self.reset_arm()
            self.printer.print("Finished put BOX")
        # TODO:
        # 
        self.move_forward()
        self.printer.print("Start Search Color Black")
        while self.step(self.timestep) != -1:
            
            if self.track_color(BLACK_COLOR):
                self.printer.print("I Find Color Black")
                break
        #
        self.PID(PURPLE_COLOR)
        pass

    def reset_to_start_point(self):
        self.turn_angle(angle=0 ,clockwise=True)
        self.move_forward()
        while self.step(self.timestep) != -1:
            if self.track_color(BLACK_COLOR):
                break
        self.PID(PURPLE_COLOR)
        self.move_forward()
        rank_color = self.get_color_name(4)
        while self.step(self.timestep) != -1:
            if self.track_color(rank_color):
                break
        self.move_distance_by_right_motor(0.4)
        self.turn_angle(angle=180)
        self.move_forward()
        while self.step(self.timestep) != -1:
            if self.track_color(PURPLE_COLOR):
                break
    # =========================== Finish ===========================
    
    def Find_and_put_box(self):
        for n in range(6):
            n += 1
            self.is_box_in_wall()
            self.passive_wait(0.3)
            self.printer.print("I Find Box in Wall")
            self.printer.print("start get Box")
            self.get_from_wall()
            self.printer.print("get_from_wall")
            self.reset_arm()
            self.printer.print("reset_arm")
            self.put_box_on_back(n)
            self.printer.print("put_box_on_back")
            self.reset_arm()
            self.printer.print("reset_arm")
            self.printer.print("finish get Box")
        
    def finish(self):
        self.move_forward()
        self.reading_color_matrix()
        self.move_forward()
        self.printer.print("Start Search Color Black")
        while self.step(self.timestep) != -1:
            if self.track_color(BLACK_COLOR):
                self.printer.print("I Find Color Black")
                break
        # 
        self.PID(PURPLE_COLOR)
        # 
        self.move_distance_by_right_motor(1)
        self.stop_at_wall()
        self.center_robot_at_wall()
        # 
        #
        # NOTE: ROBOT on PURPLE_COLOR 1
        for i in range(4):
            rank = i + 1
            rank_color = self.get_color_name(rank)
            self.printer.print(f"rank_color =>{rank} => {rank_color}")
            if(rank_color == YELLOW_COLOR):
                self.Find_and_put_box()
                # 
                self.move_distance_by_right_motor(0.1, back=True)
                self.turn_angle(-90,clockwise=True)
                self.move_distance_by_right_motor(0.3)
                # 
                self.final(rank_color)
                # 
                self.move_distance_by_right_motor(0.8)
                self.turn_angle(angle=0,clockwise=True)
                self.move_distance_by_right_motor(0.4)
                # 
                self.PID(PURPLE_COLOR)
                # 
                self.move_distance_by_right_motor(1)
                self.stop_at_wall()
                self.center_robot_at_wall()
                # 
                pass
            elif rank_color == RED_COLOR:
                self.Find_and_put_box()
                #
                self.move_distance_by_right_motor(0.2, back=True)
                self.turn_angle(230,clockwise=True)
                self.move_distance_by_right_motor(0.35)
                #
                self.final(rank_color)
                #
                self.move_distance_by_right_motor(0.9)
                self.turn_angle(angle=0,clockwise=True)
                # 
                self.PID(PURPLE_COLOR)
                # 
                self.move_distance_by_right_motor(1)
                self.stop_at_wall()
                self.center_robot_at_wall()
                pass
            elif rank_color == GREEN_COLOR:
                self.Find_and_put_box()
                # 
                self.move_distance_by_right_motor(0.2, back=True)
                self.turn_angle(-230)
                self.move_distance_by_right_motor(0.35)
                # 
                self.final(rank_color)
                #
                self.move_distance_by_right_motor(0.9)
                self.turn_angle(angle=0)
                # 
                self.PID(PURPLE_COLOR)
                # 
                self.move_distance_by_right_motor(1)
                self.stop_at_wall()
                self.center_robot_at_wall()
                pass
            elif rank_color == BLUE_COLOR:
                self.Find_and_put_box()
                # 
                self.move_distance_by_right_motor(0.1, back=True)
                self.turn_angle(90)
                self.move_distance_by_right_motor(0.3)
                # 
                self.final(rank_color)
                # 
                self.move_distance_by_right_motor(0.8)
                self.turn_angle(angle=0)
                self.move_distance_by_right_motor(0.4)
                # 
                self.PID(PURPLE_COLOR)
                # 
                self.move_distance_by_right_motor(1)
                self.stop_at_wall()
                self.center_robot_at_wall()
                pass
        
    # =========================== Finish ===========================
# NOTE:  RED BOX True # 9.89343 -5.25486 0.0124952
# 9.89343 -5.27482 0.0124917
# NOTE: YELLOW BOX 9.8894 -11.2348 0.0124917
# NOTE GREEN BOX 9.893429999048267 4.735658551276183 0.012439806389182179
# NOTE BLUE BOX 9.89344 10.7383 0.0124917
# image
# ../assets/images/all_2-01_resized.png
r = RobotController()
# r.finish()

# r.move_forward()
# r.reading_color_matrix()
# r.move_forward()
# r.printer.print("Start Search Color Black")
# while r.step(r.timestep) != -1:
#     if r.track_color(BLACK_COLOR):
#         r.printer.print("I Find Color Black")
#         break
# # 
# r.PID(PURPLE_COLOR)
# # 
# r.move_distance_by_right_motor(1)
# r.stop_at_wall()
# r.center_robot_at_wall()
# zain here
# r.is_box_in_wall()
# r.passive_wait(0.3)
# r.printer.print("I Find Box in Wall")
# r.printer.print("start get Box")
# r.get_from_wall()
# r.printer.print("get_from_wall")
# r.reset_arm()
# r.printer.print("reset_arm")
# r.put_box_on_back(1)
# r.printer.print("put_box_on_back")
# r.reset_arm()
# r.printer.print("reset_arm")
# r.printer.pritn("finish get Box")
# zain here            
# #
# r.move_distance_by_right_motor(0.9)
# r.turn_angle(angle=0,clockwise=True)
# # 
# r.PID(PURPLE_COLOR)
#NOTE: ROBOT ON wall
# -0.4832978287876744 0.0006653075926767531 0.10193771737770228
# 0 0 1 0

# 
# r.finish()
# r.go_yellow_way()
# r.final(rank_color)
# #NOTE: ROBOT on PURPLE 2 From yellow way
# # 0.5844723803486007 -0.7709898935087575 0.1019334967650635
# # -0.0010720004556421423 0.0010330315548192845 0.9999988918298008 1.5693338162233779
# r.move_distance_by_right_motor(1)
# r.turn_angle(angle=180)
# r.stop_at_wall()
# r.center_robot_at_wall()
# r.get_from_back()
# r.reset_arm()
# r.put_on_wall()
# r.reset_arm()
# r.reset_to_start_point()









# NOTE: on wall
# 0.4832310679172583 -0.0006495907970350867 0.10193349668464577
# 0.00105207657983405 2.0159167967970663e-05 -0.999999446364086 -3.141485245648484
# r.finish()











# TEST 1
# r.go_blue_way()
# r.final(BLUE_COLOR)
# # # TEST:Clean this 
# r.stop_move()

# TEST 3
# r.move_distance_by_right_motor(0.7)
# r.turn_angle(angle=180 ,clockwise=True)
# r.stop_at_wall()
# r.center_robot_at_wall()
#TEST 2
# r.turn_angle(180)
# r.find_box()
# # Put this in  find_box and rename => find_box_hold
# r.hold_box_from_ground()
# r.reset_arm()
# r.put_box_on_back()
# r.reset_arm()
# # 
# r.move_forward()
# while r.step(r.timestep) != -1:
#     if r.track_color(BLACK_COLOR):
#         break
# #
# r.PID(PURPLE_COLOR)
# r.stop_move()
# TEST 2 finish