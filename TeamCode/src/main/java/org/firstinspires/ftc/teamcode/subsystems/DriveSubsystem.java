package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.adafruit.AdafruitBNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.movement.MovementThread;
import org.firstinspires.ftc.teamcode.subsystems.movement.Movement;
import org.firstinspires.ftc.teamcode.subsystems.movement.MovementSequence;
import org.firstinspires.ftc.teamcode.util.DrivePIDCoefficients;
import org.firstinspires.ftc.teamcode.util.PIDController;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

public class DriveSubsystem extends SubsystemBase {

    public static volatile String CRASHED = "{";


    private static volatile boolean isBusy;
    private static volatile ExecutorService threadPool;
    // BLUE: facing audience = -90, facing backdrop = 90, facing away ("out") = 0, facing in = 180
    // RED: facing audience = 90, facing audience = -90, facing away ("out") = 180, facing in = 0
    private int offsetAngle;
    private final MecanumDrive controller;
    private final AdafruitBNO055IMU imu;
    private final LinearOpMode opMode;
    private final Motor leftFront;
    private final Motor rightFront;
    private final Motor leftBack;
    private final Motor rightBack;
    private Telemetry telemetry;
    public static volatile double heading=0D;
    public static volatile double imuResetValue=0D;
    public static volatile double deltaTime=0D;
    private final ElapsedTime runtime;

    // for debugging
    public static String currentMovement="";

    public DriveSubsystem(Motor leftFront, Motor rightFront, Motor leftBack, Motor rightBack, AdafruitBNO055IMU imu, LinearOpMode opMode) {
        this(leftFront, rightFront, leftBack, rightBack, imu, opMode, true, null);
    }

    public DriveSubsystem(Motor leftFront, Motor rightFront, Motor leftBack, Motor rightBack, AdafruitBNO055IMU imu, LinearOpMode opMode, boolean isRedTeam) {
        this(leftFront, rightFront, leftBack, rightBack, imu, opMode, isRedTeam, null);
    }

    public DriveSubsystem(Motor leftFront, Motor rightFront, Motor leftBack, Motor rightBack, AdafruitBNO055IMU imu, LinearOpMode opMode, boolean isRedTeam, Telemetry telemetry) {
        this.rightBack = rightBack;
        this.leftBack = leftBack;
        this.rightFront = rightFront;
        this.leftFront = leftFront;
        controller = new MecanumDrive(
                leftFront,
                rightFront,
                leftBack,
                rightBack
        );
        this.imu = imu;
        this.opMode = opMode;
        this.telemetry = telemetry;
        isBusy = false;
        threadPool = Executors.newCachedThreadPool();
        offsetAngle = isRedTeam ? 90 : -90;
        if (this.telemetry != null) {
//            this.telemetry.addData("MAX_VELOCITY", 0);
//            this.telemetry.addData("L diff",0);
//            this.telemetry.addData("R diff",0);
//
//            this.telemetry.addData("Degrees setpoint",0);
//            this.telemetry.addData("Degrees position",0);
//
//            this.telemetry.addData("LB_sp", 0);
//            this.telemetry.addData("LB_p", 0);
//            this.telemetry.addData("RB_sp", 0);
//            this.telemetry.addData("RB_p", 0);
//            this.telemetry.addData("LF_sp", 0);
//            this.telemetry.addData("LF_p", 0);
//            this.telemetry.addData("RF_sp", 0);
//            this.telemetry.addData("RF_p", 0);
//
//            // these are fine
//            this.telemetry.addData("LB_v (sp)", 0);
//            this.telemetry.addData("RB_v (sp)", 0);
//            this.telemetry.addData("LF_v (sp)", 0);
//            this.telemetry.addData("RF_v (sp)", 0);
//
//            // these might not be fine
//            this.telemetry.addData("LB Velocity", 0);
//            this.telemetry.addData("RB Velocity", 0);
//            this.telemetry.addData("LF Velocity", 0);
//            this.telemetry.addData("RF Velocity", 0);
//
//            this.telemetry.addData("LB Abs Error", 0);
//            this.telemetry.addData("RB Abs Error", 0);
//            this.telemetry.addData("LF Abs Error", 0);
//            this.telemetry.addData("RF Abs Error", 0);
//
//            telemetry.addData("LB Power", 0);
//            telemetry.addData("RB Power", 0);
//            telemetry.addData("LF Power", 0);
//            telemetry.addData("RF Power", 0);
            while (getYaw() == 0D && heading == 0D && opMode.opModeIsActive()) { // wait until imu reads heading correctly
                heading = getYaw();
            }
            heading = getYaw();

            this.telemetry.addData("HEADING", heading);
            this.telemetry.addData("YAW", getYaw());
            this.telemetry.addData("thetaDiff", 0);

            this.telemetry.update();
        }

        runtime = new ElapsedTime();
        runtime.reset();
        
    }

    /**
     * Drives with directions based on robot pov.
     *
     * @param right     How much right the robot should strafe (negative values = strafe left).
     * @param forward   How much forward the robot should move (negative values = move backwards).
     * @param turn      How much the robot should turn.
     */
    public void driveRobotCentric(double right, double forward, double turn) {
        controller.driveRobotCentric(right, forward, turn);
    }


    /**
     * Drives the motors directly with the specified motor powers.
     *
     * @param leftFrontPower     the power of the leftFront motor
     * @param rightFrontPower     the power of the rightFront motor
     * @param leftBackPower     the power of the leftBack motor
     * @param rightBackPower     the power of the rightBack motor
     */
    public void driveWithMotorPowers(double leftFrontPower, double rightFrontPower, double leftBackPower, double rightBackPower) {
        controller.driveWithMotorPowers(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
    }

    /**
     * Drives with directions based on driver pov.
     *
     * @param right     How much right the robot should strafe (negative values = strafe left).
     * @param forward   How much forward the robot should move (negative values = move backwards).
     * @param turn      How much the robot should turn.
     */
    public void driveFieldCentric(double right, double forward, double turn) {
        double yaw = getYaw();
        controller.driveFieldCentric(right, forward, turn, yaw);
    }

    /**
     * Use only for autonomous. Move a certain distance following two motion vectors for diagonal motor pairs. Drive is robot centric.
     * @param driveSpeed      Positive movement speed of the robot.
     * @param leftTicks     How many ticks the back left and front right wheels should be displaced by.
     * @param rightTicks      How many ticks the back right and front left wheels should be displaced by.
     * @param theta      The direction of movement in radians in [-π, π].
     */
    public void driveByAngularEncoder(double driveSpeed, double leftTicks, double rightTicks, double theta) {
        driveByAngularEncoder(driveSpeed, leftTicks, rightTicks, theta, false, false);
    }

    /**
     * Use only for autonomous. Move a certain distance following two motion vectors for diagonal motor pairs. Drive is robot centric.
     * @param driveSpeed      Positive movement speed of the robot.
     * @param leftTicks     How many ticks the back left and front right wheels should be displaced by.
     * @param rightTicks      How many ticks the back right and front left wheels should be displaced by.
     * @param theta      The direction of movement in radians in [-π, π].
     * @param ignoreStartVelocity    If the initial velocity should be zero.
     * @param ignoreEndVelocity    If the final velocity should be zero.
     */
    public void driveByAngularEncoder(double driveSpeed, double leftTicks, double rightTicks, double theta, boolean ignoreStartVelocity, boolean ignoreEndVelocity) {
        // check for clashing actions
        if (DriveSubsystem.getIsBusy()) {
            stopController();
            CRASHED += currentMovement+", ";
            telemetry.addData("CRASHED BECAUSE BUSY", CRASHED);
            telemetry.update();
            throw new RuntimeException("driveByAngularEncoder(): Tried to run two drive actions at once");
        }

        isBusy = true;

        // stop drivetrain at start
        if (!ignoreStartVelocity)
            stopController();

        // reset wheel encoders to 0
        leftBack.resetEncoder();
        rightBack.resetEncoder();
        leftFront.resetEncoder();
        rightFront.resetEncoder();

        // initialize variables
        double LB_start = getLBPosition(), RB_start = -getRBPosition(), LF_start = -getLFPosition(), RF_start = getRFPosition();
        double L = leftTicks, R = rightTicks;

        PIDController LB_PID = new PIDController(DrivePIDCoefficients.getDriveP(), DrivePIDCoefficients.getDriveI(), DrivePIDCoefficients.getDriveD(), LB_start);
        PIDController RB_PID = new PIDController(DrivePIDCoefficients.getDriveP(), DrivePIDCoefficients.getDriveI(), DrivePIDCoefficients.getDriveD(), RB_start);
        PIDController LF_PID = new PIDController(DrivePIDCoefficients.getDriveP(), DrivePIDCoefficients.getDriveI(), DrivePIDCoefficients.getDriveD(), LF_start);
        PIDController RF_PID = new PIDController(DrivePIDCoefficients.getDriveP(), DrivePIDCoefficients.getDriveI(), DrivePIDCoefficients.getDriveD(), RF_start);
        PIDController calculator = new PIDController();
        calculator.setVelocitySpreadProportion(DrivePIDCoefficients.VELOCITY_SPREAD_PROPORTION);

        // calculate time needed to complete entire movement
        double totalDistance = Math.hypot(L,R);
        double maxVelocity = DrivePIDCoefficients.MAX_ADJUSTED_VELOCITY;
        double[] travelTimes = calculator.calculateTravelTimes(totalDistance, maxVelocity, ignoreStartVelocity, ignoreEndVelocity);
        double firstHalfTime = travelTimes[0]; // time needed to travel the first half of the distance
        double secondHalfTime = travelTimes[1]; // time needed to travel the second half of the distance
        double totalTime = firstHalfTime + secondHalfTime;

        double startTime = runtime.seconds(); // beginning time of the movement
        double elapsedTime = 0; // will act as the independent variable t for position & velocity calculations

        double LBTurnPosDiff = 0;
        double RBTurnPosDiff = 0;
        double LFTurnPosDiff = 0;
        double RFTurnPosDiff = 0;

        while (opMode.opModeIsActive() && elapsedTime < totalTime) {

            telemetry.addData("CRASHED BECAUSE BUSY", CRASHED);
            telemetry.addData("CURRENT MOVEMENT:", DriveSubsystem.currentMovement);
//            telemetry.addData("MAX_VELOCITY", maxVelocity);

            // update values from dashboard changes
            calculator.setVelocitySpreadProportion(DrivePIDCoefficients.VELOCITY_SPREAD_PROPORTION);

            LB_PID.setPID(DrivePIDCoefficients.getDriveP(), DrivePIDCoefficients.getDriveI(), DrivePIDCoefficients.getDriveD());
            RB_PID.setPID(DrivePIDCoefficients.getDriveP(), DrivePIDCoefficients.getDriveI(), DrivePIDCoefficients.getDriveD());
            LF_PID.setPID(DrivePIDCoefficients.getDriveP(), DrivePIDCoefficients.getDriveI(), DrivePIDCoefficients.getDriveD());
            RF_PID.setPID(DrivePIDCoefficients.getDriveP(), DrivePIDCoefficients.getDriveI(), DrivePIDCoefficients.getDriveD());

            LB_PID.setErrorTolerance(DrivePIDCoefficients.getErrorTolerance_p());
            RB_PID.setErrorTolerance(DrivePIDCoefficients.getErrorTolerance_p());
            LF_PID.setErrorTolerance(DrivePIDCoefficients.getErrorTolerance_p());
            RF_PID.setErrorTolerance(DrivePIDCoefficients.getErrorTolerance_p());

            telemetry.addData("THETA", theta);

            // handle wheel velocities
            double[] velocitySetpoints = calculator.calculateVelocitySetpoints(driveSpeed, theta, elapsedTime, travelTimes, maxVelocity, ignoreStartVelocity, ignoreEndVelocity);
            double LB_v = velocitySetpoints[0], RF_v = velocitySetpoints[0], RB_v = velocitySetpoints[1], LF_v = velocitySetpoints[1];

            // handle turn correction
            double turnVelocityGain = DrivePIDCoefficients.TURN_VELOCITY_GAIN;
            double turnPositionGain = DrivePIDCoefficients.TURN_POSITION_GAIN;
            double currentHeading = getYaw();
            double thetaDiff = normalizeAngle(currentHeading - heading);
            telemetry.addData("thetaDiff", thetaDiff);
            telemetry.addData("HEADING", heading);
            telemetry.addData("YAW", getYaw());
            double absThetaDiff = Math.abs(thetaDiff);
            if (absThetaDiff < 15) {
                LB_v += turnVelocityGain * thetaDiff;
                LF_v += turnVelocityGain * thetaDiff;
                RB_v -= turnVelocityGain * thetaDiff;
                RF_v -= turnVelocityGain * thetaDiff;
                LBTurnPosDiff += turnPositionGain * thetaDiff * deltaTime;
                LFTurnPosDiff += turnPositionGain * thetaDiff * deltaTime;
                RBTurnPosDiff -= turnPositionGain * thetaDiff * deltaTime;
                RFTurnPosDiff -= turnPositionGain * thetaDiff * deltaTime;
            }

            // handle wheel positions
            double[] positionSetpoints = calculator.calculatePositionSetpoints(LB_start, RB_start, theta, elapsedTime, travelTimes, maxVelocity, ignoreStartVelocity, ignoreEndVelocity);
            double LB_sp = positionSetpoints[0], RF_sp = positionSetpoints[0], RB_sp = positionSetpoints[1], LF_sp = positionSetpoints[1];

            LB_PID.setSetPoint(LB_sp + LBTurnPosDiff);
            RB_PID.setSetPoint(RB_sp + RBTurnPosDiff);
            LF_PID.setSetPoint(LF_sp + LFTurnPosDiff);
            RF_PID.setSetPoint(RF_sp + RFTurnPosDiff);

            double LB_p = getLBPosition();
            double RB_p = -getRBPosition();
            double LF_p = getLFPosition();
            double RF_p = -getRFPosition();

            // position correction using PID
            double velocityGain = DrivePIDCoefficients.VELOCITY_GAIN;
            double LB_C = LB_PID.update(LB_p) * velocityGain;
            double RB_C = RB_PID.update(RB_p) * velocityGain;
            double LF_C = LF_PID.update(LF_p) * velocityGain;
            double RF_C = RF_PID.update(RF_p) * velocityGain;

            // works only with rawPower mode
//            telemetry.addData("LB_sp", LB_PID.getSetPoint());
//            telemetry.addData("LB_p", LB_p);
//            telemetry.addData("RB_sp", RB_PID.getSetPoint());
//            telemetry.addData("RB_p", RB_p);
//            telemetry.addData("LF_sp", LF_PID.getSetPoint());
//            telemetry.addData("LF_p", LF_p);
//            telemetry.addData("RF_sp", RF_PID.getSetPoint());
//            telemetry.addData("RF_p", RF_p);

            // these are fine
//            telemetry.addData("LB_C", LB_C);
//            telemetry.addData("RB_C", RB_C);
//            telemetry.addData("LF_C", LF_C);
//            telemetry.addData("RF_C", RF_C);

            double L_AVG_C = (Math.abs(LB_C) + Math.abs(RF_C)) / 2.0;
            double R_AVG_C = (Math.abs(RB_C) + Math.abs(LF_C)) / 2.0;

//            telemetry.addData("LB Abs Error", LB_PID.getAbsoluteDiff(LB_p));
//            telemetry.addData("RB Abs Error", RB_PID.getAbsoluteDiff(RB_p));
//            telemetry.addData("LF Abs Error", LF_PID.getAbsoluteDiff(LF_p));
//            telemetry.addData("RF Abs Error", RF_PID.getAbsoluteDiff(RF_p));

            if (!(LB_PID.getAbsoluteDiff(LB_p) + RB_PID.getAbsoluteDiff(RB_p) + LF_PID.getAbsoluteDiff(LF_p) + RF_PID.getAbsoluteDiff(RF_p) < 4 * DrivePIDCoefficients.getErrorTolerance_p())) {
                LB_v += L_AVG_C * Math.signum(LB_C);
                RB_v += R_AVG_C * Math.signum(RB_C);
                LF_v += R_AVG_C * Math.signum(LF_C);
                RF_v += L_AVG_C * Math.signum(RF_C);
            }

            // these are fine
//            telemetry.addData("LB_v (sp)", LB_v);
//            telemetry.addData("RB_v (sp)", RB_v);
//            telemetry.addData("LF_v (sp)", LF_v);
//            telemetry.addData("RF_v (sp)", RF_v);

            // these might not be fine
//            telemetry.addData("LB Velocity", getLBVelocity());
//            telemetry.addData("RB Velocity", getRBVelocity());
//            telemetry.addData("LF Velocity", getLFVelocity());
//            telemetry.addData("RF Velocity", getRFVelocity());

            // normalize velocities and drive with motor powers
            double maxTheoreticalVelocity = DrivePIDCoefficients.MAX_THEORETICAL_VELOCITY;
            double LF_power = LF_v / maxTheoreticalVelocity;
            double RF_power = RF_v / maxTheoreticalVelocity;
            double LB_power = LB_v / maxTheoreticalVelocity;
            double RB_power = RB_v / maxTheoreticalVelocity;

//            telemetry.addData("LB Power", LB_power);
//            telemetry.addData("RB Power", RB_power);
//            telemetry.addData("LF Power", LF_power);
//            telemetry.addData("RF Power", RF_power);

            driveWithMotorPowers(
                    LF_power,
                    RF_power,
                    LB_power,
                    RB_power
                    );

            telemetry.update();

            // update elapsed time
            double currentTime = runtime.seconds();
            deltaTime = currentTime - elapsedTime;
            elapsedTime = currentTime - startTime;

            isBusy = true;
        }

        // stop drivetrain at end
        if (!ignoreEndVelocity)
            stopController();
        isBusy = false;
    }

    /**
     * Use only for autonomous. Move a certain distance in ticks. Drive is robot centric.
     * @param turnSpeed      Positive rotation speed of the robot.
     * @param degrees          How many degrees the robot should turn
     */
    public void turnByIMU(double turnSpeed, double degrees) {
        // check for clashing actions
        if (DriveSubsystem.getIsBusy()) {
            stopController();
            CRASHED += currentMovement+", ";
            telemetry.addData("CRASHED BECAUSE BUSY", CRASHED);
            telemetry.update();
            throw new RuntimeException("turnByIMU(): Tried to run two drive actions at once");
        }

        isBusy = true;

        // stop drivetrain at start
        stopController();

        // reset IMU yaw to 0
        resetIMU();

        // begin action
        double start = -getYaw();
        double D = Math.abs(degrees);

        PIDController PID = new PIDController(DrivePIDCoefficients.getTurnP(), DrivePIDCoefficients.getTurnI(), DrivePIDCoefficients.getTurnD(), start);
        PIDController calculator = new PIDController();
        calculator.setVelocitySpreadProportion(DrivePIDCoefficients.ANGULAR_VELOCITY_SPREAD_PROPORTION);

        // calculate time needed to complete entire turn
        double maxVelocity = DrivePIDCoefficients.MAX_ADJUSTED_ANGULAR_VELOCITY;
        double[] travelTimes = calculator.calculateTravelTimes(D, maxVelocity, false, false);
        double firstHalfTime = travelTimes[0]; // time needed to turn the first half
        double secondHalfTime = travelTimes[1]; // time needed to turn the second half
        double totalTime = firstHalfTime + secondHalfTime;

        double startTime = runtime.seconds(); // beginning time of the turn
        double elapsedTime = 0; // will act as the independent variable t for heading & velocity calculations

        while (opMode.opModeIsActive() && elapsedTime < totalTime) {

            telemetry.addData("CRASHED BECAUSE BUSY", CRASHED);
            telemetry.addData("CURRENT MOVEMENT:", DriveSubsystem.currentMovement);
            telemetry.addData("max adjusted angular velocity", maxVelocity);

            // update values from dashboard changes
            calculator.setVelocitySpreadProportion(DrivePIDCoefficients.ANGULAR_VELOCITY_SPREAD_PROPORTION);

            PID.setPID(DrivePIDCoefficients.getTurnP(), DrivePIDCoefficients.getTurnI(), DrivePIDCoefficients.getTurnD());

            PID.setErrorTolerance(DrivePIDCoefficients.getErrorTolerance_degrees());

            // handle velocity setpoint
            double v = calculator.calculateVelocitySetpoints(turnSpeed, elapsedTime, travelTimes, maxVelocity, degrees, false, false);

            // handle heading setpoint
            double sp = normalizeAngle(calculator.calculatePositionSetpoints(start, degrees, elapsedTime, travelTimes, maxVelocity, false, false));

            PID.setSetPoint(sp);

            double p = -getYaw();

            // heading correction using PID
            double velocityGain = DrivePIDCoefficients.ANGULAR_VELOCITY_GAIN;
            double C = PID.updateError(normalizeAngle(sp - p)) * velocityGain;

            telemetry.addData("sp", PID.getSetPoint());
            telemetry.addData("p", p);
            telemetry.addData("C", C);
            telemetry.addData("Abs Error", PID.getAbsoluteDiff(p));

            if (!(PID.getAbsoluteDiff(p) < DrivePIDCoefficients.getErrorTolerance_degrees())) {
                v += C;
            }

            telemetry.addData("v (sp)", v);

            telemetry.addData("Angular Velocity", getAngularVelocity());

            double maxTheoreticalVelocity = DrivePIDCoefficients.MAX_THEORETICAL_ANGULAR_VELOCITY;
            double power = v / maxTheoreticalVelocity;

            telemetry.addData("Power", power);

            driveRobotCentric(0, 0, power);

            telemetry.update();

            // update elapsed time
            double currentTime = runtime.seconds();
            deltaTime = currentTime - elapsedTime;
            elapsedTime = currentTime - startTime;

            isBusy = true;
        }

        // brake
        stopController();
        heading = normalizeAngle(heading - degrees);
        telemetry.addData("HEADING", heading);
        telemetry.addData("YAW", getYaw());
        telemetry.update();
        isBusy = false;
    }

    /**
     * Use only for autonomous. Follow the passed MovementSequence. Drive is robot centric.
     * @param movementSequence      The MovementSequence to be followed.
     */
    public void followMovementSequence(MovementSequence movementSequence) {

        ArrayDeque<Movement> movements = movementSequence.movements.clone();

        while (opMode.opModeIsActive() && !movements.isEmpty()) {
            ArrayList<Future<?>> threadStatus = new ArrayList<>();

            // fetch all linked movements
            boolean linked = true;
            boolean needsWait = false;
            while (!movements.isEmpty() && linked && opMode.opModeIsActive()) {
                Movement movement = movements.removeFirst();
                currentMovement = "["+movement.toString()+"]";
                telemetry.addData("CRASHED BECAUSE BUSY", CRASHED);
                telemetry.addData("CURRENT MOVEMENT:", currentMovement);
                telemetry.addData("IGNORE START:", movement.ignoreStartVelocity);
                telemetry.addData("IGNORE END:", movement.ignoreEndVelocity);
                telemetry.update();
                linked = movement.linkedToNext;
                needsWait = needsWait || movement.MOVEMENT_TYPE.isDriveType || movement.MOVEMENT_TYPE.isArmType || movement.MOVEMENT_TYPE.isTurnType || movement.MOVEMENT_TYPE.isCVType;
                MovementThread thread = new MovementThread(movement);
                Future<?> status = threadPool.submit(thread);
                threadStatus.add(status); // start the MovementThread
            }

            if (needsWait)
                sleepFor(55);

            // wait until all linked movements are completed
            boolean running = true;
            while (running && opMode.opModeIsActive() || (DriveSubsystem.getIsBusy() || InDepSubsystem.getIsBusy())) {
                if (!opMode.opModeIsActive()) {
                    threadPool.shutdownNow();
                    break;
                }

                running = false;
                for (Future<?> status : threadStatus)
                    running = !status.isDone() || running; // running is only false if all threads are inactive
            }

            if (!opMode.opModeIsActive()) {
                threadPool.shutdownNow();
                break;
            }

        }

        stopController();
    }

    /**
     * @return whether or not the drivetrain is in an action.
     */
    public static boolean getIsBusy() {
        return isBusy;
    }

    /**
     * @return the current position of the robot's back left wheel in ticks.
     */
    public int getLBPosition() {
        return leftBack.getCurrentPosition();
    }

    /**
     * @return the current position of the robot's back right wheel in ticks.
     */
    public int getRBPosition() {
        return rightBack.getCurrentPosition();
    }

    /**
     * @return the current position of the robot's front left wheel in ticks.
     */
    public int getLFPosition() {
        return leftFront.getCurrentPosition();
    }

    /**
     * @return the current position of the robot's front right wheel in ticks.
     */
    public int getRFPosition() {
        return rightFront.getCurrentPosition();
    }

    /**
     * @return the current power of the robot's back left wheel.
     */
    public double getLBVelocity() {
        return leftBack.getCorrectedVelocity();
    }

    /**
     * @return the current power of the robot's back right wheel.
     */
    public double getRBVelocity() {
        return rightBack.getCorrectedVelocity();
    }

    /**
     * @return the current power of the robot's front left wheel.
     */
    public double getLFVelocity() {
        return leftFront.getCorrectedVelocity();
    }

    /**
     * @return the current power of the robot's front right wheel.
     */
    public double getRFVelocity() {
        return rightFront.getCorrectedVelocity();
    }

    /**
     * Stop the motors.
     */
    public void stopController() {
        controller.stop();
    }

    public AdafruitBNO055IMU getIMU() {
        return imu;
    }

    public void resetIMU() {
        imu.resetDeviceConfigurationForOpMode();
        imu.initialize(new BNO055IMU.Parameters());
        imuResetValue = imu.getAngularOrientation().firstAngle * 180.0 / Math.PI; // degrees
    }

    public double getYaw() {
        return normalizeAngle(imu.getAngularOrientation().firstAngle * 180.0 / Math.PI - imuResetValue + offsetAngle);
    }

    public double getAngularVelocity() {
        return imu.getAngularVelocity(AngleUnit.DEGREES).yRotationRate;
    }

    /**
     * Normalizes a given angle to [-180,180) degrees.
     * @param degrees the given angle in degrees.
     * @return the normalized angle in degrees.
     */
    private double normalizeAngle(double degrees) {
        double angle = degrees;
        while (opMode.opModeIsActive() && angle <= -180)
            angle += 360;
        while (opMode.opModeIsActive() && angle > 180)
            angle -= 360;
        return angle;
    }

    /**
     * Smart sleep with opMode running check.
     * @param ms Timeout in milliseconds.
     */
    private void sleepFor(long ms) {
        double startTime = runtime.milliseconds();
        while (opMode.opModeIsActive() && (runtime.milliseconds() - startTime < ms));
    }

}