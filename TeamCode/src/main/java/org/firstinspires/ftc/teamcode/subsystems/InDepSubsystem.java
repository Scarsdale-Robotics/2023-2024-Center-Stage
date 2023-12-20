package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.SpeedCoefficients;
import org.firstinspires.ftc.teamcode.util.PIDController;

public class InDepSubsystem extends SubsystemBase {
    private static final double Kp = 0.01;
    private static final double Ki = 0;
    private static final double Kd = 0;
    private static final double errorTolerance_p = 1.0;
    private static final double errorTolerance_v = 0.01;

    private final Motor arm1;
    private final Motor arm2;

    private final Servo leftClaw;
    private final Servo rightClaw;
    private final Servo elbow;
    private final Servo wrist;

    private static volatile boolean isBusy;
    private boolean isLeftClawOpen;
    private boolean isRightClawOpen;
    private boolean isElbowFlipped;

    private final LinearOpMode opMode;

    private Level level;
    public enum Level {
        GROUND(0, 0.0),
        BACKBOARD1(-1960,0.05),
        BACKBOARD2(-4200,0.125), //temp motor encoder values
        BACKBOARD3(-6000, 0.25);

        public final int target;
        public final double wristTarget;

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
    public enum EndEffector {
        CLAW_OPEN(0.0),
        CLAW_CLOSED(0.175),
        ELBOW_REST(0.0),
        ELBOW_FLIPPED(1.0);
        public final double servoPosition;

        EndEffector(double servoPosition) {
            this.servoPosition = servoPosition;
        }
    }

    public InDepSubsystem(Motor arm1, Motor arm2, Servo elbow, Servo wrist, Servo leftClaw, Servo rightClaw, LinearOpMode opMode) {
        // initialize objects
        this.arm1 = arm1;
        this.arm2 = arm2;

        this.rightClaw = rightClaw;
        this.leftClaw = leftClaw;

        this.elbow = elbow;
        this.wrist = wrist;

        this.opMode = opMode;

        // reset everything
        setLevel(Level.GROUND); // arm, wrist
        rest(); // elbow
        close(); // claw

        isBusy = false;

    }

    /**
     * Sets the raw power of the arm.
     */
    public void rawPower(double power) {
        arm1.motor.setPower(power);
        arm2.motor.setPower(power);
    }

    /**
     * Use only for autonomous. Change the elevation of the arm by a certain angle in ticks.
     * @param power      How fast the arm should raise (negative values = lower arm).
     * @param ticks      The angle to displace the arm by in ticks.
     */
    public void raiseByEncoder(double power, double ticks) {
        // check for clashing actions
        if (InDepSubsystem.getIsBusy()) {
            throw new RuntimeException("Tried to run two arm actions at once (arm is busy)");
        }

        // begin action
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
            rawPower(power * pidMultiplier);
            isBusy = true;
        }

        // brake
        arm1.stopMotor();
        arm2.stopMotor();
        arm1.motor.setPower(0);
        arm2.motor.setPower(0);
        isBusy = false;
    }

    /**
     * Moves the arm to a target encoder value.
     */
    public void raiseToSetPoint(double power, double setPoint) {
        new Thread(() -> raiseByEncoder(power, setPoint - getArmPosition())).start();
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
        leftClaw.setPosition(EndEffector.CLAW_OPEN.servoPosition);
        rightClaw.setPosition(EndEffector.CLAW_OPEN.servoPosition);
        isLeftClawOpen = true;
        isRightClawOpen = true;
    }

    /**
     * Closes both claws.
     */
    public void close() {
        leftClaw.setPosition(EndEffector.CLAW_CLOSED.servoPosition);
        rightClaw.setPosition(EndEffector.CLAW_CLOSED.servoPosition);
        isLeftClawOpen = false;
        isRightClawOpen = false;
    }

    /**
     * Opens the left claw.
     */
    public void openLeft() {
        leftClaw.setPosition(EndEffector.CLAW_OPEN.servoPosition);
        isLeftClawOpen = true;
    }

    /**
     * Opens the right claw.
     */
    public void openRight() {
        rightClaw.setPosition(EndEffector.CLAW_OPEN.servoPosition);
        isRightClawOpen = true;
    }

    /**
     * Closes the left claw.
     */
    public void closeLeft() {
        leftClaw.setPosition(EndEffector.CLAW_CLOSED.servoPosition);
        isLeftClawOpen = false;
    }

    /**
     * Closes the right claw.
     */
    public void closeRight() {
        rightClaw.setPosition(EndEffector.CLAW_CLOSED.servoPosition);
        isRightClawOpen = false;
    }

    /**
     * Turns the elbow servo to its flip state.
     */
    public void flip() {
        elbow.setPosition(EndEffector.ELBOW_FLIPPED.servoPosition);
        isElbowFlipped = true;
    }

    /**
     * Turns the elbow servo to its non-flipped state.
     */
    public void rest() {
        elbow.setPosition(EndEffector.ELBOW_REST.servoPosition);
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
     * @return the position of the elbow servo
     */
    public double getElbowPosition() {
        return elbow.getPosition();
    }

    /**
     * @return the position of the wrist servo
     */
    public double getWristPosition() {
        return wrist.getPosition();
    }

    /**
     * @return the position of the left claw servo
     */
    public double getLeftClawPosition() {
        return leftClaw.getPosition();
    }

    /**
     * @return the position of the right claw servo
     */
    public double getRightClawPosition() {
        return rightClaw.getPosition();
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
     * @return whether or not the arm is in an action.
     */
    public static boolean getIsBusy() {
        return isBusy;
    }

    /**
     * Resets the encoder values of both arm motors.
     */
    public void resetArmEncoder() {
        arm1.resetEncoder();
        arm2.resetEncoder();
    }
}


