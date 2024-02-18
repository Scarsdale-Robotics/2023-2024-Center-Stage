package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.util.InDepPIDCoefficients;
import org.firstinspires.ftc.teamcode.util.SpeedCoefficients;
import org.firstinspires.ftc.teamcode.util.PIDController;

public class InDepSubsystem extends SubsystemBase {
    private final Motor arm1;
    private final Motor arm2;

    private final Servo leftClaw;
    private final Servo rightClaw;
    private final ServoImplEx elbow;
    private final ServoImplEx wrist;

    private static volatile boolean isBusy;
    private boolean isLeftClawOpen;
    private boolean isRightClawOpen;
    private boolean isElbowFlipped;
    private double delta;

    private final LinearOpMode opMode;
    private MultipleTelemetry telemetry;

    private Level level = Level.GROUND;

    public enum Level {
        GROUND(0, 0.31, false),
        BACKBOARD_HIGH(1261,0.39, true),
        BACKBOARD_MID(2150,0.34, true), // tuned values
        BACKBOARD_LOW(2330, 0.27, true);

        public final int target;
        public final double wristTarget;
        public final boolean elbowFlipped;

        Level(int target, double wristTarget, boolean elbowFlipped) {
            this.target = target;
            this.wristTarget = wristTarget;
            this.elbowFlipped = elbowFlipped;
        }

        public Level nextAbove() {
            if (this == GROUND) return BACKBOARD_HIGH;
            if (this == BACKBOARD_HIGH) return BACKBOARD_MID;
            if (this == BACKBOARD_MID) return BACKBOARD_LOW;
            return GROUND;
        }

        public Level nextBelow() {
            if (this == BACKBOARD_LOW) return BACKBOARD_MID;
            if (this == BACKBOARD_MID) return BACKBOARD_HIGH;
            if (this == BACKBOARD_HIGH) return GROUND;
            return BACKBOARD_LOW;
        }
    }
    public enum EndEffector {
        LEFT_CLAW_OPEN(0.8),
        LEFT_CLAW_CLOSED(0.55),
        RIGHT_CLAW_OPEN(0.0),
        RIGHT_CLAW_CLOSED(0.25),
        ELBOW_REST(0.0),
        ELBOW_FLIPPED(0.99);
        public final double servoPosition;

        EndEffector(double servoPosition) {
            this.servoPosition = servoPosition;
        }
    }
    public final int ELBOW_TURN_TICKS = 5038;

    public InDepSubsystem(Motor arm1, Motor arm2, ServoImplEx elbow, ServoImplEx wrist, Servo leftClaw, Servo rightClaw, LinearOpMode opMode) {
        this(arm1,arm2,elbow,wrist,leftClaw,rightClaw,opMode,null);
    }

    public InDepSubsystem(Motor arm1, Motor arm2, ServoImplEx elbow, ServoImplEx wrist, Servo leftClaw, Servo rightClaw, LinearOpMode opMode, MultipleTelemetry telemetry) {
        // initialize objects
        this.arm1 = arm1;
        this.arm2 = arm2;

        this.rightClaw = rightClaw;
        this.leftClaw = leftClaw;

        this.elbow = elbow;
        this.wrist = wrist;

        this.opMode = opMode;
        this.telemetry = telemetry;
        this.delta = 0;

//         reset everything

//        setLevel(Level.GROUND); // arm, wrist
//        rest(); // elbow
        close(); // claw

        isBusy = false;

    }

    public Level getLevelBelow() {
        int armPos = getLeftArmPosition();
        if (armPos < Level.BACKBOARD_HIGH.target+delta) {
            return Level.GROUND;
        }
        if (armPos < Level.BACKBOARD_MID.target+delta) {
            return Level.BACKBOARD_HIGH;
        }
        if (armPos < Level.BACKBOARD_LOW.target+delta) {
            return Level.BACKBOARD_MID;
        }
        return Level.BACKBOARD_LOW;
    }

    /**
     * Calculates the power coefficient for the arm motors based on the arm position. (https://www.desmos.com/calculator/8qjoopxfda)
     */
    private double calculatePowerCoefficient(double armPos) {
        double relMax = 500;

        if (armPos <= relMax)
            return Math.max(2.0/3.0,
                    1 /
                            (Math.pow(
                                    (armPos-relMax) /
                                            (double)381,2) + 1)
            );
        else
            return Math.max(2.0/3.0,
                    1 /
                            (Math.pow(
                                    (armPos-relMax) /
                                            (double)1525,2) + 1)
            );
    }

    /**
     * Sets the raw power of the arm.
     */
    public void rawPower(double power) {
        // TODO: add ranges
        int armPos = getLeftArmPosition();

        // TODO: desmos graph stuff need to be adjusted
        double K_power = calculatePowerCoefficient(armPos-delta);

        // set bounds
        if (armPos < delta && power < 0 && !opMode.gamepad1.a) {
            arm1.motor.setPower(0);
            arm2.motor.setPower(0);
        } else {
            if (EndgameSubsystem.droneReleased && opMode.gamepad2.left_trigger > 0.5 && opMode.gamepad2.right_trigger > 0.5) {
                arm1.motor.setPower(power);
                arm2.motor.setPower(power);
            } else {
                arm1.motor.setPower(K_power * power);
                arm2.motor.setPower(K_power * power);
            }
        }
        opMode.telemetry.addData("level: ", level);
        opMode.telemetry.addData("nxt below: ", getLevelBelow());
        opMode.telemetry.addData("elbow pos: ", elbow.getPosition());
        opMode.telemetry.addData("level wrist target: ", level.wristTarget);

//        if (level != getLevelBelow()) {
//            level = getLevelBelow();
//            if (level.elbowFlipped) {
//                flip();
//            } else {
//                rest();
//            }
//        }

        // TODO: desmos graph stuff need to be adjusted
        if (level != getLevelBelow()) {
            level = getLevelBelow();
        }
        setElbowPosition(calculateElbowPosition(armPos-delta));

        wrist.setPosition(level.wristTarget);

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
        double L_startEncoder = arm1.motor.getCurrentPosition();
        double L_setPoint = L_startEncoder + ticks;
        PIDController L_PID = new PIDController(InDepPIDCoefficients.getKp(), InDepPIDCoefficients.getKi(), InDepPIDCoefficients.getKd(), L_setPoint);

        while (
                opMode.opModeIsActive() && !(
                        Math.abs(L_setPoint - getLeftArmPosition()) < InDepPIDCoefficients.getErrorTolerance_p() &&
                                Math.abs(getLeftArmVelocity()) < InDepPIDCoefficients.getErrorTolerance_v())
        ) {
            L_PID.setPID(InDepPIDCoefficients.getKp(), InDepPIDCoefficients.getKi(), InDepPIDCoefficients.getKd());

            double L_K = L_PID.update(getLeftArmPosition());

            arm1.motor.setPower(power * L_K);
            arm2.motor.setPower(power * L_K);

//            setWristPosition(0.25);

            if (telemetry != null) {
                telemetry.addData("CURRENT MOVEMENT:", DriveSubsystem.currentMovement);
                telemetry.addData("L Arm pos", getLeftArmPosition());
                telemetry.addData("L Setpoint", L_setPoint);
                telemetry.update();
            }
            if (getLeftArmPosition() >= Level.BACKBOARD_LOW.target+delta) {
                wrist.setPosition(Level.BACKBOARD_LOW.wristTarget);
            } else if (getLeftArmPosition() >= Level.BACKBOARD_MID.target+delta) {
                wrist.setPosition(Level.BACKBOARD_MID.wristTarget);
            } else if (getLeftArmPosition() >= Level.BACKBOARD_HIGH.target+delta) {
                wrist.setPosition(Level.BACKBOARD_HIGH.wristTarget);
            } else if(getLeftArmPosition() >= Level.GROUND.target+delta) {
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
        openLeft();
        openRight();
    }

    /**
     * Closes both claws.
     */
    public void close() {
        closeLeft();
        closeRight();
    }

    /**
     * Opens the left claw.
     */
    public void openLeft() {
        leftClaw.setPosition(EndEffector.LEFT_CLAW_OPEN.servoPosition);
        isLeftClawOpen = true;
        if (isRightClawOpen) {
            SpeedCoefficients.setMode(SpeedCoefficients.MoveMode.MODE_FAST);
        }
    }

    /**
     * Opens the right claw.
     */
    public void openRight() {
        rightClaw.setPosition(EndEffector.RIGHT_CLAW_OPEN.servoPosition);
        isRightClawOpen = true;
        if (isLeftClawOpen) {
            SpeedCoefficients.setMode(SpeedCoefficients.MoveMode.MODE_FAST);
        }
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
     * Calculates the elbow's position based on the arm's position. (https://www.desmos.com/calculator/8qjoopxfda)
     */
    private double calculateElbowPosition(double armPos) {
        double lowerBound = Level.BACKBOARD_HIGH.target-50;
        double m_elbow = (EndEffector.ELBOW_FLIPPED.servoPosition - EndEffector.ELBOW_REST.servoPosition) / ((double)Level.BACKBOARD_HIGH.target - lowerBound);

        if (armPos <= lowerBound)
            return EndEffector.ELBOW_REST.servoPosition;
        else if (armPos <= (double)Level.BACKBOARD_HIGH.target)
            return m_elbow*armPos - m_elbow*500 + EndEffector.ELBOW_REST.servoPosition;
        else
            return EndEffector.ELBOW_FLIPPED.servoPosition;
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
        delta = getLeftArmPosition();
    }

    public double getDelta() {
        return delta;
    }
}

