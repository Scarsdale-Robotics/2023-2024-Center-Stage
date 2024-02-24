package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.InDepPIDCoefficients;
import org.firstinspires.ftc.teamcode.util.SpeedCoefficients;
import org.firstinspires.ftc.teamcode.util.PIDController;

public class InDepSubsystem extends SubsystemBase {
    private final Motor leftArm;
    private final Motor rightArm;

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
    private ElapsedTime runtime;

    private Level level = Level.GROUND;

    public enum Level {
        GROUND(0, 0.58, false),
        BACKBOARD_HIGH(1261,0.55, true),
        BACKBOARD_MID(2150,0.58, true), // tuned values
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
        LEFT_CLAW_OPEN(0.75),
        LEFT_CLAW_CLOSED(0.55),
        RIGHT_CLAW_OPEN(0.05),
        RIGHT_CLAW_CLOSED(0.25),
        ELBOW_REST(0.763),
        ELBOW_FLIPPED(0.262);
        public final double servoPosition;

        EndEffector(double servoPosition) {
            this.servoPosition = servoPosition;
        }
    }
    public final int ELBOW_TURN_TICKS = 5038;

    public InDepSubsystem(Motor leftArm, Motor rightArm, ServoImplEx elbow, ServoImplEx wrist, Servo leftClaw, Servo rightClaw, LinearOpMode opMode) {
        this(leftArm, rightArm,elbow,wrist,leftClaw,rightClaw,opMode,null);
    }

    public InDepSubsystem(Motor leftArm, Motor rightArm, ServoImplEx elbow, ServoImplEx wrist, Servo leftClaw, Servo rightClaw, LinearOpMode opMode, MultipleTelemetry telemetry) {
        // initialize objects
        this.leftArm = leftArm;
        this.rightArm = rightArm;

        this.rightClaw = rightClaw;
        this.leftClaw = leftClaw;

        this.elbow = elbow;
        this.wrist = wrist;

        this.opMode = opMode;
        this.telemetry = telemetry;
        this.delta = 0;

        this.runtime = new ElapsedTime();
        runtime.reset();

//         reset everything

//        setLevel(Level.GROUND); // arm, wrist
//        rest(); // elbow

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

        double K_power = calculatePowerCoefficient(armPos-delta);

        // set bounds
        if (armPos < delta && power < 0 && !opMode.gamepad1.a) {
            leftArm.motor.setPower(0);
            rightArm.motor.setPower(0);
        } else {
            if (EndgameSubsystem.droneReleased && opMode.gamepad2.left_trigger > 0.5 && opMode.gamepad2.right_trigger > 0.5) {
                leftArm.motor.setPower(power);
                rightArm.motor.setPower(power);
            } else {
                leftArm.motor.setPower(K_power * power);
                rightArm.motor.setPower(K_power * power);
            }
        }
        opMode.telemetry.addData("level: ", level);
        opMode.telemetry.addData("nxt below: ", getLevelBelow());
        opMode.telemetry.addData("elbow pos: ", elbow.getPosition());
        opMode.telemetry.addData("level wrist target: ", level.wristTarget);

        if (level != getLevelBelow()) {
            level = getLevelBelow();
        }

        setElbowPosition(calculateElbowPosition(armPos-delta));
        // TODO: add ada's equations?
        setWristPosition(calculateWristPosition(armPos-delta));

        opMode.telemetry.addData("chicken: ", "nugget");
        opMode.telemetry.addData("elbowPos", elbow.getPosition());
        opMode.telemetry.addData("wristPos", wrist.getPosition());
    }

    /**
     * Use only for autonomous. Change the angle of the arm by a certain amount of ticks.
     * @param armSpeed      How fast the arm should raise.
     * @param ticks      The angle to displace the arm by in ticks (negative values = lower arm).
     */
    public void raiseByEncoder(double armSpeed, double ticks) {

        // check for clashing actions
        if (InDepSubsystem.getIsBusy()) {
            throw new RuntimeException("Tried to run two arm actions at once (arm is busy)");
        }

        // stop arm at start
        stopMotors();

        // reset arm encoders to 0
        leftArm.resetEncoder();
        rightArm.resetEncoder();

        // begin action
        double L_start = getLeftArmPosition(), R_start = getRightArmPosition();
        double D = ticks;

        PIDController L_PID = new PIDController(InDepPIDCoefficients.getKp(), InDepPIDCoefficients.getKi(), InDepPIDCoefficients.getKd(), L_start);
        PIDController R_PID = new PIDController(InDepPIDCoefficients.getKp(), InDepPIDCoefficients.getKi(), InDepPIDCoefficients.getKd(), R_start);
        PIDController calculator = new PIDController();
        calculator.setVelocitySpreadProportion(InDepPIDCoefficients.VELOCITY_SPREAD_PROPORTION);

        L_PID.setIntegralBounds(-1, 1);
        L_PID.setOutputBounds(-1, 1);

        R_PID.setIntegralBounds(-1, 1);
        R_PID.setOutputBounds(-1, 1);

        // calculate time needed to complete entire arm movement
        double maxVelocity = InDepPIDCoefficients.MAX_VELOCITY;
        double[] travelTimes = calculator.calculateTravelTimes(D, maxVelocity, false, false);
        double firstHalfTime = travelTimes[0]; // time needed to turn the first half of the angle
        double secondHalfTime = travelTimes[1]; // time needed to turn the second half of the angle
        double totalTime = firstHalfTime + secondHalfTime;

        double startTime = runtime.seconds(); // beginning time of the arm movement
        double elapsedTime = 0; // will act as the independent variable t for position & velocity calculations

        while (opMode.opModeIsActive() && elapsedTime < totalTime) {

            calculator.setVelocitySpreadProportion(InDepPIDCoefficients.VELOCITY_SPREAD_PROPORTION);

            L_PID.setPID(InDepPIDCoefficients.getKp(), InDepPIDCoefficients.getKi(), InDepPIDCoefficients.getKd());
            R_PID.setPID(InDepPIDCoefficients.getKp(), InDepPIDCoefficients.getKi(), InDepPIDCoefficients.getKd());

            L_PID.setErrorTolerance(InDepPIDCoefficients.getErrorTolerance_p());
            R_PID.setErrorTolerance(InDepPIDCoefficients.getErrorTolerance_p());

            // handle arm velocity setpoint
            double velocitySetpoint = calculator.calculateVelocitySetpoints(armSpeed, elapsedTime, travelTimes, maxVelocity, false, false);
            double L_v = velocitySetpoint;
            double R_v = velocitySetpoint;

            // handle arm position setpoints
            double L_sp = calculator.calculatePositionSetpoints(L_start, elapsedTime, travelTimes, maxVelocity, false, false);
            double R_sp = calculator.calculatePositionSetpoints(R_start, elapsedTime, travelTimes, maxVelocity, false, false);

            L_PID.setSetPoint(L_sp);
            R_PID.setSetPoint(R_sp);

            double L_p = getLeftArmPosition();
            double R_p = getRightArmPosition();

            // position correction using PID
            double velocityGain = InDepPIDCoefficients.VELOCITY_GAIN;
            double L_C = L_PID.update(L_p) * velocityGain;
            double R_C = R_PID.update(R_p) * velocityGain;

            telemetry.addData("L_sp", L_PID.getSetPoint());
            telemetry.addData("L_p", L_p);
            telemetry.addData("R_sp", R_PID.getSetPoint());
            telemetry.addData("R_p", R_p);
            telemetry.addData("L_C", L_C);
            telemetry.addData("R_C", R_C);

            telemetry.addData("L Abs Error", L_PID.getAbsoluteDiff(L_p));
            telemetry.addData("R Abs Error", R_PID.getAbsoluteDiff(R_p));

            if (!(L_PID.getAbsoluteDiff(L_p) + R_PID.getAbsoluteDiff(R_p) < 2 * InDepPIDCoefficients.getErrorTolerance_p())) {
                L_v += L_C;
                R_v += R_C;
            }

            telemetry.addData("L_v (sp)", L_v);
            telemetry.addData("R_v (sp)", R_v);

            telemetry.addData("L Velocity", getLeftArmVelocity());
            telemetry.addData("R Velocity", getRightArmVelocity());

            // normalize velocities and turn arm with motor powers
            double theoreticalMaxVelocity = InDepPIDCoefficients.MAX_VELOCITY;
            double L_power = L_v / theoreticalMaxVelocity;
            double R_power = R_v / theoreticalMaxVelocity;

            telemetry.addData("L Power", L_power);
            telemetry.addData("R Power", R_power);

            leftArm.motor.setPower(L_power);
            rightArm.motor.setPower(R_power);

            telemetry.update();

            // update elapsed time
            elapsedTime = runtime.seconds() - startTime;

            isBusy = true;
        }

        // brake arms at end
        stopMotors();
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
     * Stops the arm motors.
     */
    public void stopMotors() {
        leftArm.stopMotor();
        rightArm.stopMotor();
        leftArm.motor.setPower(0);
        rightArm.motor.setPower(0);
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
     * Calculates the wrist's position based on the arm's position
     */
    private double calculateWristPosition(double armPos) {
        double lowerBound = 1000;
        double[][] points = new double[][] {
                {1867, 0.46},
                {1955, 0.47},
                {2089, 0.49},
                {2132, 0.52},
                {2264, 0.56},
                {2302, 0.57},
                {2366, 0.58}
        };

        if (armPos < lowerBound)
            return 0.58;
        else {
            for (int i = 0; i < points.length; i++) {
                if (armPos < points[i][0]) {
                    if (i == 0)
                        return points[0][1];

                    double[] previousPoint = points[i-1], currentPoint = points[i];
                    double slope = (currentPoint[1] - previousPoint[1]) / (currentPoint[0] - previousPoint[0]);
                    return previousPoint[1] + slope * (armPos - previousPoint[0]);
                }
            }
            return points[points.length-1][1];
        }
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
        return leftArm.motor.getCurrentPosition();
    }

    /**
     * @return the power of the left arm motor
     */
    public double getLeftArmVelocity() {
        return leftArm.motor.getPower();
    }

    /**
     * @return the position of the right arm motor, in ticks.
     */
    public int getRightArmPosition() {
        return rightArm.motor.getCurrentPosition();
    }

    /**
     * @return the power of the right arm motor
     */
    public double getRightArmVelocity() {
        return rightArm.motor.getPower();
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

