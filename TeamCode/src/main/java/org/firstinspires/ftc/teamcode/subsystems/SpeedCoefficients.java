package org.firstinspires.ftc.teamcode.subsystems;

public class SpeedCoefficients {
    public static final int MODE_SLOW = 0;
    public static final int MODE_FAST = 1;

    private static int moveMode = MODE_FAST;

    public static void setMode(int targetMode) {
        moveMode = targetMode;
    }

    public static double getForwardSpeed() {
        double SLOW_FORWARD_SPEED = 0.5;
        double FAST_FORWARD_SPEED = 1;
        return (moveMode == MODE_FAST) ? FAST_FORWARD_SPEED : SLOW_FORWARD_SPEED;
    }

    public static double getStrafeSpeed() {
        double SLOW_STRAFE_SPEED = 0.5;
        double FAST_STRAFE_SPEED = 1;
        return (moveMode == MODE_FAST) ? FAST_STRAFE_SPEED : SLOW_STRAFE_SPEED;
    }

    public static double getTurnSpeed() {
        double SLOW_TURN_SPEED = 0.5;
        double FAST_TURN_SPEED = 1;
        return (moveMode == MODE_FAST) ? FAST_TURN_SPEED : SLOW_TURN_SPEED;
    }
}
