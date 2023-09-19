package org.firstinspires.ftc.teamcode.subsystems;

public class SpeedCoefficients {
    public final int MODE_SLOW = 0;
    public final int MODE_FAST = 1;

    private int moveMode = MODE_FAST;

    public void setMode(int targetMode) {
        moveMode = targetMode;
    }

    public double getForwardSpeed() {
        double SLOW_FORWARD_SPEED = 0;
        double FAST_FORWARD_SPEED = 0;
        return (moveMode == MODE_FAST) ? FAST_FORWARD_SPEED : SLOW_FORWARD_SPEED;
    }

    public double getStrafeSpeed() {
        double SLOW_STRAFE_SPEED = 0;
        double FAST_STRAFE_SPEED = 0;
        return (moveMode == MODE_FAST) ? FAST_STRAFE_SPEED : SLOW_STRAFE_SPEED;
    }

    public double getTurnSpeed() {
        double SLOW_TURN_SPEED = 0;
        double FAST_TURN_SPEED = 0;
        return (moveMode == MODE_FAST) ? FAST_TURN_SPEED : SLOW_TURN_SPEED;
    }
}
