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
    public boolean ignoreStartVelocity;
    public boolean ignoreEndVelocity;

    Movement(MovementType type, double forward, double strafe, double turn, double elevation, double servoPosition, long wait) {
        this.MOVEMENT_TYPE = type;
        this.INCHES_FORWARD = forward;
        this.INCHES_STRAFE = strafe;
        this.DEGREES_TURN = turn;
        this.DEGREES_ELEVATION = elevation;
        this.SERVO_POSITION = servoPosition;
        this.WAIT = wait;
        linkedToNext = false;
        ignoreStartVelocity = false;
        ignoreEndVelocity = false;
    }

    public void linkWithNext() {
        linkedToNext = true;
    }

    public void setIgnoredStartVelocity() {
        ignoreStartVelocity = true;
    }

    public void setIgnoredEndVelocity() {
        ignoreEndVelocity = true;
    }

    public String toString() {
        switch (MOVEMENT_TYPE) {
            case FORWARD_RIGHT:
                return "FORWARD_RIGHT";
            case FORWARD_LEFT:
                return "FORWARD_LEFT";
            case BACKWARD_RIGHT:
                return "BACKWARD_RIGHT";
            case BACKWARD_LEFT:
                return "BACKWARD_LEFT";
            case FORWARD:
                return "FORWARD";
            case BACKWARD:
                return "BACKWARD";
            case STRAFE_LEFT:
                return "STRAFE_LEFT";
            case STRAFE_RIGHT:
                return "STRAFE_RIGHT";
            case TURN_LEFT:
                return "TURN_LEFT";
            case TURN_RIGHT:
                return "TURN_RIGHT";
            case DELAY:
                return "DELAY";
            case CLOSE_LEFT_CLAW:
                return "CLOSE_LEFT_CLAW";
            case OPEN_LEFT_CLAW:
                return "OPEN_LEFT_CLAW";
            case CLOSE_RIGHT_CLAW:
                return "CLOSE_RIGHT_CLAW";
            case OPEN_RIGHT_CLAW:
                return "OPEN_RIGHT_CLAW";
            case FLIP_ELBOW:
                return "FLIP_ELBOW";
            case REST_ELBOW:
                return "REST_ELBOW";
            case SET_WRIST:
                return "SET_WRIST";
            case LOWER_ARM:
                return "LOWER_ARM";
            case RAISE_ARM:
                return "RAISE_ARM";
            case WHITE_PXL_ALIGN:
                return "WHITE_PXL_ALIGN";
            case APRIL_TAG_ALIGN:
                return "APRIL_TAG_ALIGN";
            default:
                return "null";
        }

    }

}
