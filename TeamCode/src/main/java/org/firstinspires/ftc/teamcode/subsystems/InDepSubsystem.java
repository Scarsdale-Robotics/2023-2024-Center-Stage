package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.SpeedCoefficients;

public class InDepSubsystem extends SubsystemBase {
    private static final double Kp = 0.01;
    private static final double Ki = 0;
    private static final double Kd = 0;
    private final double errorTolerance_p = 1.0;
    private final double errorTolerance_v = 0.01;
    private final double CLAW_OPEN_POS = 0;
    private final double CLAW_CLOSED_POS = 0.175;
    private final double ELBOW_TURN_180 = 1;
    private final double ELBOW_TURN_REGULARPOS = 0;

    private final LinearOpMode opMode;
    private final Motor arm1;
    private final Motor arm2;
    private final Servo rightClaw;
    private final Servo leftClaw;
    private final Servo elbow;
    private Level level;
    private final Servo wrist;
    private boolean isLeftClawOpen;
    private boolean isRightClawOpen;
    private boolean isElbowFlipped;

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
        close();
        setLevel(Level.GROUND);
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
     * Use only for autonomous. Change the elevation of the arm by a certain angle in ticks.
     * @param power      How fast the arm should raise (negative values = lower arm).
     * @param ticks      The angle to displace the arm by in ticks.
     */
    public void raiseByEncoder(double power, double ticks) {
        double startEncoder = arm1.motor.getCurrentPosition(); // arm1 is our reference for encoders
        double setPoint = startEncoder + ticks;
        double pidMultiplier;

        PIDController pidController = new PIDController(Kp, Ki, Kd, setPoint);

        while (
                opMode.opModeIsActive() &&
                Math.abs(setPoint-getArmPosition()) > errorTolerance_p &&
                Math.abs(getArmVelocity()) > errorTolerance_v
        ) {
            pidMultiplier = pidController.update(getArmPosition());
            arm1.motor.setPower(power * pidMultiplier);
            arm2.motor.setPower(power * pidMultiplier);
        }

        arm1.stopMotor();
        arm2.stopMotor();
        arm1.motor.setPower(0);
        arm2.motor.setPower(0);
    }

    /**
     * Moves the arm to a target encoder value.
     */
    public void raiseToSetPoint(double power, double setPoint) {
        raiseByEncoder(power, setPoint - getArmPosition());
    }

    /**
     * Sets the position of the arm to zero.
     */
    public void resetArm() {
        raiseToSetPoint(SpeedCoefficients.getArmSpeed(), 0);
    }

    /**
     * Moves the arm to a target level.
     */
    public void setLevel(Level level) {
        wrist.setPosition(level.wristTarget);
        raiseToSetPoint(SpeedCoefficients.getArmSpeed(), level.target);
    }

    /**
     * Raises the arm to the next available level.
     */
    public void raiseArm() {
        level = level.nextAbove();
        setLevel(level);
    }

    /**
     * Lowers the arm to the next available level.
     */
    public void lowerArm() {
        level = level.nextBelow();
        setLevel(level);
    }

    /**
     * Opens both claws.
     */
    public void open() {
        leftClaw.setPosition(CLAW_OPEN_POS);
        rightClaw.setPosition(CLAW_OPEN_POS);
        isLeftClawOpen = true;
        isRightClawOpen = true;
    }

    /**
     * Closes both claws.
     */
    public void close() {
        leftClaw.setPosition(CLAW_CLOSED_POS);
        rightClaw.setPosition(CLAW_CLOSED_POS);
        isLeftClawOpen = false;
        isRightClawOpen = false;
    }

    /**
     * Opens the left claw.
     */
    public void openLeft() {
        leftClaw.setPosition(CLAW_OPEN_POS);
        isLeftClawOpen = true;
    }

    /**
     * Opens the right claw.
     */
    public void openRight() {
        rightClaw.setPosition(CLAW_OPEN_POS);
        isRightClawOpen = true;
    }

    /**
     * Closes the left claw.
     */
    public void closeLeft() {
        leftClaw.setPosition(CLAW_CLOSED_POS);
        isLeftClawOpen = false;
    }

    /**
     * Closes the right claw.
     */
    public void closeRight() {
        rightClaw.setPosition(CLAW_CLOSED_POS);
        isRightClawOpen = false;
    }

    /**
     * Turns the elbow servo to its flip state.
     */
    public void flip() {
        elbow.setPosition(ELBOW_TURN_180);
        isElbowFlipped = true;
    }

    /**
     * Turns the elbow servo to its unflipped state.
     */
    public void rest() {
        elbow.setPosition(ELBOW_TURN_REGULARPOS);
        isElbowFlipped = false;
    }

    /**
     * @return the position of the arm motor, in ticks.
     */
    public int getArmPosition() {
        return arm1.motor.getCurrentPosition();
    }

    /**
     * @return the power of the arm motor
     */
    public double getArmVelocity() {
        return arm1.motor.getPower();
    }

    /**
     * @return the position of the wrist servo
     */
    public double getWristPosition() {
        return wrist.getPosition();
    }

    /**
     * @return true if the left claw is open, otherwise false if it is closed.
     */
    public boolean getIsLeftClawOpen() {
        return isLeftClawOpen;
    }

    /**
     * @return true if the right claw is open, otherwise false if it is closed.
     */
    public boolean getIsRightClawOpen() {
        return isRightClawOpen;
    }

    /**
     * @return true if the elbow is flipped, otherwise false if it is rested.
     */
    public boolean getIsElbowFlipped() {
        return isElbowFlipped;
    }

    /**
     * Resets the encoder values of both arm motors.
     */
    public void resetArmEncoder() {
        arm1.resetEncoder();
        arm2.resetEncoder();
    }
}


