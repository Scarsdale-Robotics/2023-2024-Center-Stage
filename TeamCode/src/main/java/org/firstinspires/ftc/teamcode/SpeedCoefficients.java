package org.firstinspires.ftc.teamcode;

public class SpeedCoefficients {
    public static MoveMode moveMode;
    public enum MoveMode {
        MODE_SLOW(0),
        MODE_FAST(1);
        int moveMode;
        MoveMode(int moveMode) {this.moveMode = moveMode;}
    }

    public static void setMode(MoveMode targetMode) {
        moveMode = targetMode;
    }

    public static double getForwardSpeed() {
        double SLOW_FORWARD_SPEED = 0.5;
        double FAST_FORWARD_SPEED = 1;
        return moveMode.equals(MoveMode.MODE_FAST) ? FAST_FORWARD_SPEED : SLOW_FORWARD_SPEED;
    }

    public static double getStrafeSpeed() {
        double SLOW_STRAFE_SPEED = 0.5;
        double FAST_STRAFE_SPEED = 1;
        return moveMode.equals(MoveMode.MODE_FAST) ? FAST_STRAFE_SPEED : SLOW_STRAFE_SPEED;
    }

    public static double getTurnSpeed() {
        double SLOW_TURN_SPEED = 0.5;
        double FAST_TURN_SPEED = 1;
        return moveMode.equals(MoveMode.MODE_FAST) ? FAST_TURN_SPEED : SLOW_TURN_SPEED;
    }

    public static double getArmSpeed() {
        double SLOW_ARM_SPEED = 0.5;
        double FAST_ARM_SPEED = 1;
        return moveMode.equals(MoveMode.MODE_FAST) ? FAST_ARM_SPEED : SLOW_ARM_SPEED;
    }
}
