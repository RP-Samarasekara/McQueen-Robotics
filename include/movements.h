

#include "encoders.h"
#include "motors.h"
#include "config.h"

class Movement {
public:
    Movement(Motors& m, Encoders& e) : motors(m), encoders(e) {}

    // Turn 90 degrees: right = true for right, false for left
    void turn90(bool right = true) {
        float startAngle = encoders.robotAngle(); // read current angle
        float turn_speed = 30;                    // motor power percentage

        if (right) {
            motors.set_left_motor_percentage(turn_speed);
            motors.set_right_motor_percentage(-turn_speed);
        } else {
            motors.set_left_motor_percentage(-turn_speed);
            motors.set_right_motor_percentage(turn_speed);
        }

        // wait until robot has rotated 90 degrees
        while (abs(encoders.robotAngle() - startAngle) < 90.0) {
            // optionally call encoders.update() here if not using ISR
        }

        motors.stop(); // stop motors
    }

private:
    Motors& motors;
    Encoders& encoders;
};
