package org.firstinspires.ftc.teamcode.subsystems.movement;

public class Movement {
    // TYPES OF MOVEMENTS:
    // 0 - forward
    // 1 - backward
    // 2 - left
    // 3 - right
    // 4 - turn left
    // 5 - turn right
    // 6 - delay/no movement
    // 7 - close claw
    // 8 - open claw
    // 9 - change elevation
    public int MOVEMENT_TYPE;
    public double INCHES_FORWARD, INCHES_STRAFE, DEGREES_TURN;
    public long WAIT;
    public int ELEVATION;

    Movement(int type, double forward, double strafe, double turn, long wait, int elevation) {
        this.MOVEMENT_TYPE = type;
        this.INCHES_FORWARD = forward;
        this.INCHES_STRAFE = strafe;
        this.DEGREES_TURN = turn;
        this.WAIT = wait;
        this.ELEVATION = elevation;
    }
}
