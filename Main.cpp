#include <pigpio.h>
#include <iostream>
#include <chrono>
#include <vector>

UPPER_LEG = 4
LOWER_LEG = 5

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

void go_to_pos(std::pair<int, int> point) {
    // some IK math
    int x;
    int y;
    set_servo_angle(UPPER_LEG, x);
    set_servo_angle(LOWER_LEG, y);
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
}
