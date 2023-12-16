package org.firstinspires.ftc.teamcode.subsystems.movement;

public class Movement {
    public enum MovementType {
        FORWARD (1.0,0.0,0.0,0.0),
        BACKWARD (-1.0,0.0,0.0,0.0),
        STRAFE_LEFT (0.0,-1.0,0.0,0.0),
        STRAFE_RIGHT (0.0,1.0,0.0,0.0),
        TURN_LEFT (0.0,0.0,-1.0,0.0),
        TURN_RIGHT (0.0,0.0,1.0,0.0),
        DELAY (0.0,0.0,0.0,0.0),
        CLOSE_CLAW (0.0,0.0,0.0,0.0),
        OPEN_CLAW (0.0,0.0,0.0,0.0),
        LOWER_ARM (0.0,0.0,0.0,1.0),
        RAISE_ARM (0.0,0.0,0.0,-1.0);

        public double K_forward;
        public double K_strafe;
        public double K_turn;
        public double K_elevation;
        MovementType(double K_forward, double K_strafe, double K_turn, double K_elevation) {
            this.K_forward = K_forward;
            this.K_strafe = K_strafe;
            this.K_turn = K_turn;
            this.K_elevation = K_elevation;
        }
    }
    public MovementType MOVEMENT_TYPE;
    public double INCHES_FORWARD, INCHES_STRAFE, DEGREES_TURN, DEGREES_ELEVATION;
    public long WAIT;

    Movement(MovementType type, double forward, double strafe, double turn, double elevation, long wait) {
        this.MOVEMENT_TYPE = type;
        this.INCHES_FORWARD = forward;
        this.INCHES_STRAFE = strafe;
        this.DEGREES_TURN = turn;
        this.DEGREES_ELEVATION = elevation;
        this.WAIT = wait;
    }
}
