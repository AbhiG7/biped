import numpy
import matplotlib.pyplot as plt
import math
import numpy as np
import time

class Robot():
    upper_leg_length = 120.0
    lower_leg_length = 120.0
    shoulder_motor_angle = 300.0 # degrees
    knee_motor_angle = 240.0 # degrees

# given length of end effector from hip joint, return angles of knee and shoulder joints
def go_to_pos(height, robot):
    a = robot.lower_leg_length
    b = robot.upper_leg_length
    cosa = (b**2 + height**2 - a**2) / (2*b*height)
    shoulder_angle = math.acos(cosa) # radians
    shoulder_angle_deg = shoulder_angle * (180/math.pi)
    knee_angle_deg = 180 - shoulder_angle_deg*2
    knee_angle_calc = -90 + shoulder_angle_deg - (180 - knee_angle_deg)
    return 90 - shoulder_angle_deg, -knee_angle_calc

def ik(coord):
    # given target x y, what are the angles to go to
    x = coord[0]
    y = coord[1]
    A = 120 # leg length 1
    B = 120 # leg length 2
    term = (x**2 + y**2 + A**2 - B**2)/math.sqrt((2*A*x)**2 + (2*A*y)**2)
    theta1 = (math.atan((2*A*y)/(2*A*x)) + math.acos(term),
              math.atan((2*A*y)/(2*A*x)) - math.acos(term))
    theta2 = (math.atan((y-A*math.sin(theta1[0]))/(x-A*math.cos(theta1[0]))) - theta1[0],
              math.atan((y-A*math.sin(theta1[1]))/(x-A*math.cos(theta1[1]))) - theta1[1])
    print((theta1[0]*(180/math.pi), theta1[1]*(180/math.pi)),
          (theta2[0]*(180/math.pi), theta2[1]*(180/math.pi)))
    theta1_deg = theta1[0] * (180/math.pi)
    theta2_deg = theta2[0] * (180/math.pi)
    return theta1_deg, theta2_deg


def gen_line(start, length, angle_deg):
    # Convert angle to radians
    angle_rad = np.radians(angle_deg)

    # Calculate the end point (x1, y1)
    x1 = start[0] + length * np.cos(angle_rad)
    y1 = start[1] + length * np.sin(angle_rad)

    return ([start[0], x1], [start[1], y1], angle_deg)

def main():
    myRobot = Robot()
    endpoints = [(1,-120.0), (-20,-100.0), (-70,-85.5), (-20,-72.3), (20,-60.0), (70,-100.0), (30,-150.0), (1,-185.5)]
    # plt.figure()
    # for s in endpoints:
    #     setpoint = s[1]
    #     plt.clf()
    #     res = go_to_pos(setpoint, myRobot);
    #     myRobot.shoulder_motor_angle = res[0]
    #     myRobot.knee_motor_angle = res[1]
    #     print(f"Going to: {setpoint}. Shoulder angle: {res[0]}. Knee angle: {res[1]}")
    #     upper_leg = gen_line((0,0), myRobot.upper_leg_length, myRobot.shoulder_motor_angle)
    #     plt.plot(upper_leg[0], upper_leg[1], marker='o', label=f'Line at {upper_leg[2]}°')
    #     lower_leg = gen_line([upper_leg[0][1], upper_leg[1][1]], myRobot.lower_leg_length, myRobot.knee_motor_angle)
    #     plt.plot(lower_leg[0], lower_leg[1], marker='x', label=f'Line at {lower_leg[2]}°')
    #     leg_dist = gen_line((0,0), setpoint, 90)
    #     plt.plot(leg_dist[0], leg_dist[1], marker='x', label=f'Line at {leg_dist[2]}°')
    #     plt.xlim(-60,180)
    #     plt.ylim(-200, 10)
    #     plt.draw()
    #     plt.pause(0.2)
    # plt.show()
    plt.figure()
    eppe = [(1,-120)]
    for s in eppe:
        plt.clf()
        res = ik(s)
        myRobot.shoulder_motor_angle = res[0]
        myRobot.knee_motor_angle = res[1]
        upper_leg = gen_line((0,0), myRobot.upper_leg_length, myRobot.shoulder_motor_angle)
        plt.plot(upper_leg[0], upper_leg[1], marker='o', label=f'Line at {upper_leg[2]}°')
        lower_leg = gen_line([upper_leg[0][1], upper_leg[1][1]], myRobot.lower_leg_length, myRobot.knee_motor_angle)
        plt.plot(lower_leg[0], lower_leg[1], marker='x', label=f'Line at {lower_leg[2]}°')
        leg_dist = gen_line((0,0), s[1], 90)
        plt.plot(leg_dist[0], leg_dist[1], marker='x', label=f'Line at {leg_dist[2]}°')
        plt.xlim(-100,200)
        plt.ylim(-200, 100)
        plt.draw()
        plt.pause(0.5)
    plt.show()

        # print(f"Endpoint: {s}, Res: {res}")

if __name__ == "__main__":
    main()