#include <pigpio.h>
#include <iostream>
#include <chrono>
#include <vector>
#include <pair>
#include <cmath>

UPPER_LEG = 4
LOWER_LEG = 5

class Robot {
public:
    float upperLegLength = 120.0; // mm
    float lowerLegLength = 120.0; // mm
    float upperMotorPos = 90.0; // starting pos
    float lowerMotorPos = 90.0; //starting pos
    float hipMotorPos = 90.0; // starting pos
};

void set_servo_angle(int servo, int angle) {
    // Convert the angle to pulse width in microseconds (500-2500 μs)
    int pwm = 500 + (angle / 180.0) * 2000;
    gpioServo(servo, pwm);
}

void test() {
    set_servo_angle(SERVO_PIN, 0);
    std::sleep(1);
    set_servo_angle(SERVO_PIN, 180);
    std::sleep(1);
    set_servo_angle(SERVO_PIN, 0);
    std::sleep(1);
}

// given length of end effector from hip joint, return angles of knee and shoulder joints
std::pair<int> go_to_pos(float height, Robot r) {
    // some IK math
    float a = r.lowerLegLength;
    float b = r.upperLegLength
    float cosa = (b**2 + height**2 - a**2) / (2*b*height);
    float shoulderAngle = acos(cosa); // radians
    float kneeAngle = M_PI - (shoulderAngle * 2); // radians
    float shoulderAngleDeg = shoulderAngle * (180/M_PI); // deg
    float kneeAngleDeg = kneeAngle * (180/M_PI);
    return std::pair(shoulderAngleDeg, kneeAngleDeg);
}

void followPath(std::vector<int> path) {
    // for a given path (an equation of a path)
    // (x-2)^2 = x^2-4x+4
    path = {1, -4, 4};
    // generate points along that path at set intervals
    std::vector<std::pair<float, float>> points;
    // go to each of those points
    for (auto p : points) {
        go_to_pos(p);
    }
}


int main() {
    /*
    // Setup
    if (gpioInitialize() < 0) {
        std::cerr << "Failed to initialize gpio.\n";
        return 1;
    }
    gpioSetMode(UPPER_LEG, PI_OUTPUT);
    gpioSetMode(LOWER_LEG, PI_OUTPUT);

    auto start = std::high_resolution_clock::now()
    while(std::duration_cast<std::seconds>(std::high_resolution_clock::now() - start) < 30) {
        followPath();
    
    }
    */
   Robot myRobot();
   float startPoint = 120.0; // mm
   std::vector<float> endpoints = {100.0, 85.5, 72.3, 60.0, 100.0, 150.0, 185.5};
   for (auto setpoint : endpoints) {
    std::pair res = go_to_pos(setpoint, myRobot);
    std::cout << "Going to: " << setpoint << ". Shoulder angle: " << res.first << 
   }
}
