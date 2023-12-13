package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.SpeedCoefficients;

public class InDepSubsystem extends SubsystemBase {
    private final double CLAW_OPEN_POS = 0;
    private final double CLAW_CLOSED_POS = 0.175;
    private final double ELBOW_TURN_180 = 1;
    private final double ELBOW_TURN_REGULARPOS = 0;
    private final double errorTolerance = 200;

    private final LinearOpMode opMode;
    private final Motor arm1;
    private final Motor arm2;
    private final Servo rightClaw;
    private final Servo leftClaw;
    private final Servo elbow;
    private Level level;
    private final Servo wrist;
    private boolean isClawOpen;
    private boolean isElbowTurned;

    public enum Level {
        GROUND(0, 0),
        BACKBOARD1(-1960,0.05),
        BACKBOARD2(-4200,0.125), //temp motor encoder values
        BACKBOARD3(-6000, 0.25);

        public int target;
        public double wristTarget;

        Level(int target, double wristTarget) {
            this.target = target;
            this.wristTarget = wristTarget;
        }

        public Level nextAbove() {
            if (this == GROUND) return BACKBOARD1;
            else if (this == BACKBOARD1) return BACKBOARD2;
            else return BACKBOARD3; // For MEDIUM and HIGH
        }

        public Level nextBelow() {
            if (this == BACKBOARD3) return BACKBOARD2;
            else if (this == BACKBOARD2) return BACKBOARD1;
            else return GROUND;
        }
    }
    public InDepSubsystem(Motor arm1, Motor arm2, Servo rightClaw, Servo leftClaw, Servo wrist, Servo elbow, LinearOpMode opMode, Telemetry telemetry) {
        // initialize objects
        this.arm1 = arm1;
        this.arm2 = arm2;

        this.rightClaw = rightClaw;
        this.leftClaw = leftClaw;

        this.elbow = elbow;

//        this.rightClaw = rightClaw;
//        this.leftClaw = leftClaw;
        this.wrist = wrist;
        this.opMode = opMode;

        // reset the arm and claw states
        closeClaws();
        setLevel(Level.GROUND);
    }
    public void stopAllMotors() {
        arm1.stopMotor();
        arm2.stopMotor();
    }
    public void setTPAllMotors(int target) {
        arm1.setTargetPosition(target);
        arm2.setTargetPosition(target);
    }
    public void setPowerAllMotors(double power) {
        arm1.motor.setPower(power);
        arm2.motor.setPower(power);
    }
    public void setSpeedAllMotors(double output) {
        arm1.set(output);
        arm2.set(output);
    }
    public void resetEncoderAllMotors()
    {
        arm1.resetEncoder();
        arm2.resetEncoder();
    }

    /**
     * @return the position of the arm motor, in ticks.
     */
    public int getArmPosition() {
        return arm1.motor.getCurrentPosition();
    }

    /**
     * @return true if the claw is open, otherwise false if it is closed.
     */
    //public double getWristPosition(){return wrist.}
    public boolean getIsOpen() {
        return isClawOpen;
    }

    private int targetPosition = Level.GROUND.target;
    private boolean movingToTarget = false;
    public void setMoveAsyncToPosition(int position) {
        targetPosition = position;
        movingToTarget = true;
    }

    public void tickMoveAsyncToPosition(double power, boolean lowerBoundDisabled) {
        int ERROR = 20;
        int diff = arm1.motor.getCurrentPosition() - targetPosition;
        if (movingToTarget && Math.abs(diff) <= ERROR) {
//            rawPowerBase(power * (diff < 0 ? 1 : -1), lowerBoundDisabled);
        }
    }

//    private void rawPowerBase(double power, boolean lowerBoundDisabled) {
//        int armPos = arm.motor.getCurrentPosition();
//
//        // set bounds
//        if (armPos>=0 && power>0 && !lowerBoundDisabled) { // more down is more positive
//            arm.motor.setPower(0);
//        } else {
//            arm.motor.setPower(power);
//        }
//
//        if (armPos <= Level.BACKBOARD2.target) {
//            wrist.setPosition(Level.BACKBOARD2.wristTarget);
//        } else if (armPos <= Level.BACKBOARD1.target) {
//            wrist.setPosition(Level.BACKBOARD1.wristTarget);
//        } else {
//            wrist.setPosition(Level.GROUND.wristTarget);
//        }
//    }

    /**
     * sets the raw power of the arm.
     */
    public void rawPower(double power, boolean lowerBoundDisabled) {
        movingToTarget = false;
//        rawPowerBase(power, lowerBoundDisabled);
    }

    /**
     * gets the level immediately above or immediately below the current position
     * @param getAbove whether to get the above level or below level
     * @return the nearby level specified (by getAbove)
     */
    private Level getNearbyLevel(boolean getAbove) {
        double armPos = arm1.getCurrentPosition();
        if (armPos > Level.BACKBOARD2.target)
            return getAbove ? Level.BACKBOARD2 : Level.BACKBOARD1;
        else if (armPos > Level.BACKBOARD1.target)
            return getAbove ? Level.BACKBOARD2 : Level.GROUND;
        else
            return getAbove ? Level.BACKBOARD1 : Level.GROUND;
    }

    /**
     * Raises the arm.
     */
//    public void raiseArm() {
//        setLevel(getNearbyLevel(true));
//    }

    /**
     * Lowers the arm.
     */
//    public void lowerArm() {
//        setLevel(getNearbyLevel(false));
//    }

    public void setLevel(Level level) {
        setTPAllMotors(level.target);
        wrist.setPosition(level.wristTarget);
        setSpeedAllMotors(SpeedCoefficients.getArmSpeed());
    }

    public void raiseArm() {
        setLevel(level.nextAbove());
    }

    public void lowerArm() {
        setLevel(level.nextBelow());
    }

    public void setOriginalPosOfElbow() {
        elbow.setPosition(ELBOW_TURN_REGULARPOS);
        isElbowTurned = false;
    }
    public void set180DegreePosOfElbow() {
        elbow.setPosition(ELBOW_TURN_180);
        isElbowTurned = true;
    }

    public boolean getElbowTurn() {return isElbowTurned;}


    public void resetArmEncoder() {
        resetEncoderAllMotors();
    }

    /**
     * Opens the claw.
     */
    public void openClaws() {
        leftClaw.setPosition(CLAW_OPEN_POS);
        rightClaw.setPosition(CLAW_OPEN_POS);
        isClawOpen = true;
    }
    public void openLeftClaw() {
        leftClaw.setPosition(CLAW_OPEN_POS);
    }
    public void openRightClaw() {
        rightClaw.setPosition(CLAW_OPEN_POS);
    }

    /**
     * Closes the claw.
     */
    public void closeClaws() {
        leftClaw.setPosition(CLAW_CLOSED_POS);
        rightClaw.setPosition(CLAW_CLOSED_POS);
        isClawOpen = false;
    }
    public void closeLeftClaw() {
        leftClaw.setPosition(CLAW_CLOSED_POS);
    }
    public void closeRightClaw() {
        rightClaw.setPosition(CLAW_CLOSED_POS);
    }

    /**
     * Use only for autonomous. Change the elevation of the arm by a certain angle in ticks.
     * @param power      How fast the arm should raise (negative values = lower arm).
     * @param ticks      The angle to displace the arm by in ticks.
     */
    public void raiseByEncoder(double power, double ticks) {
        double startEncoder = arm1.motor.getCurrentPosition();
        // do we need PID here?
        // if placing on the backboard during auto then we should atleast use kP and kI

        while (opMode.opModeIsActive() && Math.abs(arm1.motor.getCurrentPosition() - startEncoder) < ticks) {
            setPowerAllMotors(power);
        }

        stopAllMotors();

        setPowerAllMotors(0);
    }

    public void resetArm() {
        int target = 0;
        setTPAllMotors(target);
        setSpeedAllMotors(SpeedCoefficients.getArmSpeed());

        // wait until reached target within errorTolerance
        while (opMode.opModeIsActive() && !(Math.abs(target-arm1.getCurrentPosition()) < errorTolerance) );

        // stop when reached
        stopAllMotors();
        setSpeedAllMotors(0);
    }
}


