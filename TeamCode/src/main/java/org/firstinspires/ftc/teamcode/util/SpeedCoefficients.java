package org.firstinspires.ftc.teamcode.util;

public class SpeedCoefficients {
    public static MoveMode moveMode = MoveMode.MODE_FAST;
    public enum MoveMode {
        MODE_SLOW(0),
        MODE_FAST(1);
        int mode;
        MoveMode(int moveMode) {this.mode = moveMode;}
    }

    public static void setMode(MoveMode targetMode) {
        moveMode = targetMode;
    }
    public static int getMode() {
        return moveMode == MoveMode.MODE_SLOW ? 0 : 1;

    }

    public static double getForwardSpeed() {
        double SLOW_FORWARD_SPEED = 0.22;
        double FAST_FORWARD_SPEED = 1;
        return moveMode.mode==MoveMode.MODE_FAST.mode ? FAST_FORWARD_SPEED : SLOW_FORWARD_SPEED;
    }

    public static double getStrafeSpeed() {
        double SLOW_STRAFE_SPEED = 0.33;
        double FAST_STRAFE_SPEED = 1;
        return moveMode.mode==MoveMode.MODE_FAST.mode ? FAST_STRAFE_SPEED : SLOW_STRAFE_SPEED;
    }

    public static double getTurnSpeed() {
        double SLOW_TURN_SPEED = 0.22;
        double FAST_TURN_SPEED = 0.77;
        return moveMode.mode==MoveMode.MODE_FAST.mode ? FAST_TURN_SPEED : SLOW_TURN_SPEED;
    }

    public static double getArmSpeed() {
        double SLOW_ARM_SPEED = 0.7;
        double FAST_ARM_SPEED = 1;
        return moveMode.mode==MoveMode.MODE_FAST.mode ? FAST_ARM_SPEED : SLOW_ARM_SPEED;
    }

    public static double getAutonomousDriveSpeed() {
        return 1;
    }

    public static double getAutonomousTurnSpeed() {
        return 0.5;
    }

    public static double getAutonomousArmSpeed() {return 0.8;}

}
