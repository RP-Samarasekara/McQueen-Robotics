

#include "encoders.h"
#include "motors.h"
#include "config.h"

class Movement {
public:
    Movement(Motors& m, Encoders& e) : motors(m), encoders(e) {}

    // Turn 90 degrees: right = true for right, false for left
    void turn90(bool right = true) {
       
    }

private:
    Motors& motors;
    Encoders& encoders;
};
