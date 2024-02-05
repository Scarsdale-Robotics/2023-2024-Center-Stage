package org.firstinspires.ftc.teamcode.subsystems.movement;

public class Movement {
    public enum MovementType {
        FORWARD_RIGHT (1.0,1.0,0.0,0.0, true, false, false, false, false),
        FORWARD_LEFT (1.0,-1.0,0.0,0.0, true, false, false, false, false),
        BACKWARD_RIGHT (-1.0,1.0,0.0,0.0, true, false, false, false, false),
        BACKWARD_LEFT (-1.0,-1.0,0.0,0.0, true, false, false, false, false),
        FORWARD (1.0,0.0,0.0,0.0, true, false, false, false, false),
        BACKWARD (-1.0,0.0,0.0,0.0, true, false, false, false, false),
        STRAFE_LEFT (0.0,-1.0,0.0,0.0, true, false, false, false, false),
        STRAFE_RIGHT (0.0,1.0,0.0,0.0, true, false, false, false, false),
        TURN_LEFT (0.0,0.0,-1.0,0.0, false, true, false, false, false),
        TURN_RIGHT (0.0,0.0,1.0,0.0, false, true, false, false, false),
        DELAY (0.0,0.0,0.0,0.0, false, false, false, false, false),
        CLOSE_LEFT_CLAW(0.0,0.0,0.0,0.0, false, false, true, false, false),
        OPEN_LEFT_CLAW(0.0,0.0,0.0,0.0, false, false, true, false, false),
        CLOSE_RIGHT_CLAW(0.0,0.0,0.0,0.0, false, false, true, false, false),
        OPEN_RIGHT_CLAW(0.0,0.0,0.0,0.0, false, false, true, false, false),
        REST_ELBOW(0.0,0.0,0.0,0.0, false, false, false, false, false),
        FLIP_ELBOW(0.0,0.0,0.0,0.0, false, false, false, false, false),
        SET_WRIST(0.0,0.0,0.0,0.0, false, false, false, false, false),
        LOWER_ARM(0.0,0.0,0.0,-1.0, false, false, false, true, false),
        RAISE_ARM(0.0,0.0,0.0,1.0, false, false, false, true, false),
        WHITE_PXL_ALIGN(1.0, 1.0, 1.0, 0.0, false, false, false, false, true),
        APRIL_TAG_ALIGN(1.0, 1.0, 1.0, 0.0, false, false, false, false, true);

        public double SGN_forward;
        public double SGN_strafe;
        public double SGN_turn;
        public double SGN_elevation;
        public boolean isDriveType, isTurnType, isClawType, isArmType, isCVType;
        MovementType(double SGN_forward, double SGN_strafe, double SGN_turn, double SGN_elevation, boolean isDriveType, boolean isTurnType, boolean isClawType, boolean isArmType, boolean isCVType) {
            this.SGN_forward = SGN_forward;
            this.SGN_strafe = SGN_strafe;
            this.SGN_turn = SGN_turn;
            this.SGN_elevation = SGN_elevation;
            this.isDriveType = isDriveType;
            this.isTurnType = isTurnType;
            this.isClawType = isClawType;
            this.isArmType = isArmType;
            this.isCVType = isCVType;
        }
    }
    public MovementType MOVEMENT_TYPE;
    public double INCHES_FORWARD, INCHES_STRAFE, DEGREES_TURN, DEGREES_ELEVATION;
    public double SERVO_POSITION;
    public long WAIT;
    public boolean linkedToNext;

    Movement(MovementType type, double forward, double strafe, double turn, double elevation, double servoPosition, long wait) {
        this.MOVEMENT_TYPE = type;
        this.INCHES_FORWARD = forward;
        this.INCHES_STRAFE = strafe;
        this.DEGREES_TURN = turn;
        this.DEGREES_ELEVATION = elevation;
        this.SERVO_POSITION = servoPosition;
        this.WAIT = wait;
        linkedToNext = false;
    }

    public void linkWithNext() {
        linkedToNext = true;
    }
}
