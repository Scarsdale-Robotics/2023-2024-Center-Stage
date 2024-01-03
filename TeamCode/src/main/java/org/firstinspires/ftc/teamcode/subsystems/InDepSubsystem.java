package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.InDepPIDCoefficients;
import org.firstinspires.ftc.teamcode.util.SpeedCoefficients;
import org.firstinspires.ftc.teamcode.util.PIDController;

public class InDepSubsystem extends SubsystemBase {
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
    private MultipleTelemetry telemetry;

    private Level level = Level.GROUND;

    public enum Level {
        GROUND(0, 0.25, 0.84),
        BACKBOARD1(5083,0.35, 0.17),
        BACKBOARD2(6789,0.4, 0.17), //temp motor encoder values
        BACKBOARD3(8111, 0.4, 0.17);

        public final int target;
        public final double wristTarget;
        public final double elbowTarget;

        Level(int target, double wristTarget, double elbowTarget) {
            this.target = target;
            this.wristTarget = wristTarget;
            this.elbowTarget = elbowTarget;
        }

        public Level nextAbove() {
            if (this == GROUND) return BACKBOARD1;
            if (this == BACKBOARD1) return BACKBOARD2;
            if (this == BACKBOARD2) return BACKBOARD3;
            return GROUND;
        }

        public Level nextBelow() {
            if (this == BACKBOARD3) return BACKBOARD2;
            if (this == BACKBOARD2) return BACKBOARD1;
            if (this == BACKBOARD1) return GROUND;
            return BACKBOARD3;
        }
    }
    public enum EndEffector {
        LEFT_CLAW_OPEN(0.6),
        LEFT_CLAW_CLOSED(0.21),
        RIGHT_CLAW_OPEN(0.25),
        RIGHT_CLAW_CLOSED(0.60),
        ELBOW_REST(0.80),
        ELBOW_FLIPPED(0.13);
        public final double servoPosition;

        EndEffector(double servoPosition) {
            this.servoPosition = servoPosition;
        }
    }
    public final int ELBOW_TURN_TICKS = 5038;

    public InDepSubsystem(Motor arm1, Motor arm2, Servo elbow, Servo wrist, Servo leftClaw, Servo rightClaw, LinearOpMode opMode) {
        this(arm1,arm2,elbow,wrist,leftClaw,rightClaw,opMode,null);
    }

    public InDepSubsystem(Motor arm1, Motor arm2, Servo elbow, Servo wrist, Servo leftClaw, Servo rightClaw, LinearOpMode opMode, MultipleTelemetry telemetry) {
        // initialize objects
        this.arm1 = arm1;
        this.arm2 = arm2;

        this.rightClaw = rightClaw;
        this.leftClaw = leftClaw;

        this.elbow = elbow;
        this.wrist = wrist;

        this.opMode = opMode;
        this.telemetry = telemetry;

        // reset everything

//        setLevel(Level.GROUND); // arm, wrist
//        rest(); // elbow
//        close(); // claw

        isBusy = false;

    }

    public Level getLevelBelow() {
        int armPos = getLeftArmPosition();
        if (armPos < Level.BACKBOARD1.target) {
            return Level.GROUND;
        }
        if (armPos < Level.BACKBOARD2.target) {
            return Level.BACKBOARD1;
        }
        if (armPos < Level.BACKBOARD3.target) {
            return Level.BACKBOARD2;
        }
        return Level.BACKBOARD3;
    }

    /**
     * Sets the raw power of the arm.
     */
    public void rawPower(double power) {
        // TODO: add ranges
        int armPos = getLeftArmPosition();

        // set bounds
        if (armPos < 0 && power < 0 && !opMode.gamepad1.a) {
            arm1.motor.setPower(0);
            arm2.motor.setPower(0);
        } else {
            arm1.motor.setPower(power);
            arm2.motor.setPower(power);
        }
        opMode.telemetry.addData("level: ", level);
        opMode.telemetry.addData("nxt below: ", getLevelBelow());
        opMode.telemetry.addData("level elbow target: ", level.elbowTarget);
        opMode.telemetry.addData("level wrist target: ", level.wristTarget);
        if (level != getLevelBelow())
        {
            level = getLevelBelow();
            elbow.setPosition(level.elbowTarget);
            wrist.setPosition(level.wristTarget);
        }
        opMode.telemetry.addData("chicken: ", "nugget");
        opMode.telemetry.addData("elbowPos", elbow.getPosition());
        opMode.telemetry.addData("wristPos", wrist.getPosition());
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
        double L_startEncoder = arm1.motor.getCurrentPosition(), R_startEncoder = arm2.motor.getCurrentPosition();
        double L_setPoint = L_startEncoder + ticks, R_setPoint = R_startEncoder + ticks;
        PIDController L_PID = new PIDController(InDepPIDCoefficients.getKp(), InDepPIDCoefficients.getKi(), InDepPIDCoefficients.getKd(), L_setPoint);
        PIDController R_PID = new PIDController(InDepPIDCoefficients.getKp(), InDepPIDCoefficients.getKi(), InDepPIDCoefficients.getKd(), R_setPoint);

        while (
                opMode.opModeIsActive() && !(
                        Math.abs(L_setPoint - getLeftArmPosition()) < InDepPIDCoefficients.getErrorTolerance_p() &&
                        Math.abs(getLeftArmVelocity()) < InDepPIDCoefficients.getErrorTolerance_v()) && !(
                        Math.abs(R_setPoint - getRightArmPosition()) < InDepPIDCoefficients.getErrorTolerance_p() &&
                        Math.abs(getRightArmVelocity()) < InDepPIDCoefficients.getErrorTolerance_v())
        ) {
            L_PID.setPID(InDepPIDCoefficients.getKp(), InDepPIDCoefficients.getKi(), InDepPIDCoefficients.getKd());
            R_PID.setPID(InDepPIDCoefficients.getKp(), InDepPIDCoefficients.getKi(), InDepPIDCoefficients.getKd());

            double L_K = L_PID.update(getLeftArmPosition());
            double R_K = R_PID.update(getRightArmPosition());

            arm1.motor.setPower(power * L_K);
            arm2.motor.setPower(power * R_K);

//            setWristPosition(0.25);

            if (telemetry != null) {
                telemetry.addData("L Arm pos", getLeftArmPosition());
                telemetry.addData("L Setpoint", L_setPoint);
                telemetry.addData("R Arm pos", getRightArmPosition());
                telemetry.addData("R Setpoint", R_setPoint);
                telemetry.update();
            }
            if (getLeftArmPosition() >= Level.BACKBOARD3.target) {
                wrist.setPosition(Level.BACKBOARD3.wristTarget);
            } else if (getLeftArmPosition() >= Level.BACKBOARD2.target) {
                wrist.setPosition(Level.BACKBOARD2.wristTarget);
            } else if (getLeftArmPosition() >= Level.BACKBOARD1.target) {
                wrist.setPosition(Level.BACKBOARD1.wristTarget);
            } else if(getLeftArmPosition() >= Level.GROUND.target) {
                wrist.setPosition(Level.GROUND.wristTarget);
            } else {
                wrist.setPosition(0.25);
            }

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
        new Thread(() -> raiseByEncoder(power, setPoint - getLeftArmPosition())).start();
    }

    /**
     * Sets the position of the arm to zero.
     */
    public void resetArm() {
        raiseToSetPoint(SpeedCoefficients.getArmSpeed(), 0);
    }

    /**
     * Sets the wrist servo to the passed position.
     */
    public void setWristPosition(double servoPosition) {
        wrist.setPosition(servoPosition);
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
     * Sets the left claw servo to the passed position.
     */
    public void setLeftClawPosition(double servoPosition) {
        leftClaw.setPosition(servoPosition);
    }

    /**
     * Sets the right claw servo to the passed position.
     */
    public void setRightClawPosition(double servoPosition) {
        rightClaw.setPosition(servoPosition);
    }

    /**
     * Opens both claws.
     */
    public void open() {
        leftClaw.setPosition(EndEffector.LEFT_CLAW_OPEN.servoPosition);
        rightClaw.setPosition(EndEffector.RIGHT_CLAW_OPEN.servoPosition);
        isLeftClawOpen = true;
        isRightClawOpen = true;
    }

    /**
     * Closes both claws.
     */
    public void close() {
        leftClaw.setPosition(EndEffector.LEFT_CLAW_CLOSED.servoPosition);
        rightClaw.setPosition(EndEffector.RIGHT_CLAW_CLOSED.servoPosition);
        isLeftClawOpen = false;
        isRightClawOpen = false;
    }

    /**
     * Opens the left claw.
     */
    public void openLeft() {
        leftClaw.setPosition(EndEffector.LEFT_CLAW_OPEN.servoPosition);
        isLeftClawOpen = true;
    }

    /**
     * Opens the right claw.
     */
    public void openRight() {
        rightClaw.setPosition(EndEffector.RIGHT_CLAW_OPEN.servoPosition);
        isRightClawOpen = true;
    }

    /**
     * Closes the left claw.
     */
    public void closeLeft() {
        leftClaw.setPosition(EndEffector.LEFT_CLAW_CLOSED.servoPosition);
        isLeftClawOpen = false;
    }

    /**
     * Closes the right claw.
     */
    public void closeRight() {
        rightClaw.setPosition(EndEffector.RIGHT_CLAW_CLOSED.servoPosition);
        isRightClawOpen = false;
    }

    /**
     * Sets the elbow servo to the passed position.
     */
    public void setElbowPosition(double servoPosition) {
        elbow.setPosition(servoPosition);
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
     * @return the position of the left arm motor, in ticks.
     */
    public int getLeftArmPosition() {
        return arm1.motor.getCurrentPosition();
    }

    /**
     * @return the power of the left arm motor
     */
    public double getLeftArmVelocity() {
        return arm1.motor.getPower();
    }

    /**
     * @return the position of the right arm motor, in ticks.
     */
    public int getRightArmPosition() {
        return arm2.motor.getCurrentPosition();
    }

    /**
     * @return the power of the right arm motor
     */
    public double getRightArmVelocity() {
        return arm2.motor.getPower();
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


