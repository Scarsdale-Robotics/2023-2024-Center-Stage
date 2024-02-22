package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.adafruit.AdafruitBNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.movement.MovementThread;
import org.firstinspires.ftc.teamcode.subsystems.movement.Movement;
import org.firstinspires.ftc.teamcode.subsystems.movement.MovementSequence;
import org.firstinspires.ftc.teamcode.util.DrivePIDCoefficients;
import org.firstinspires.ftc.teamcode.util.PIDController;
import org.firstinspires.ftc.teamcode.util.SpeedCoefficients;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

public class DriveSubsystem extends SubsystemBase {

    private static volatile boolean isBusy;
    private static volatile ExecutorService threadPool;
    // facing audience = -90, facing backdrop = 90, facing away ("out") = 0, facing in = 180
    private int offsetAngle = -90;
    private final MecanumDrive controller;
    private final AdafruitBNO055IMU imu;
    private final LinearOpMode opMode;
    private final Motor leftFront;
    private final Motor rightFront;
    private final Motor leftBack;
    private final Motor rightBack;
    private Telemetry telemetry;
    public static volatile double heading=0D;
    private final ElapsedTime runtime;

    // for debugging
    public static String currentMovement="";

    public DriveSubsystem(Motor leftFront, Motor rightFront, Motor leftBack, Motor rightBack, AdafruitBNO055IMU imu, LinearOpMode opMode) {
        this(leftFront, rightFront, leftBack, rightBack, imu, opMode, null);
    }

    public DriveSubsystem(Motor leftFront, Motor rightFront, Motor leftBack, Motor rightBack, AdafruitBNO055IMU imu, LinearOpMode opMode, Telemetry telemetry) {
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
        while (getYaw() == 0D && heading == 0D && opMode.opModeIsActive()) { // wait until imu reads heading correctly
            heading = getYaw();
        }
        heading = getYaw();
        isBusy = false;
        threadPool = Executors.newCachedThreadPool();

        if (this.telemetry != null) {
            telemetry.addData("MAX_VELOCITY", 0);
            this.telemetry.addData("L diff",0);
            this.telemetry.addData("R diff",0);

            this.telemetry.addData("Degrees setpoint",0);
            this.telemetry.addData("Degrees position",0);

            this.telemetry.addData("LB_sp", 0);
            this.telemetry.addData("LB_p", 0);
            this.telemetry.addData("RB_sp", 0);
            this.telemetry.addData("RB_p", 0);
            this.telemetry.addData("LF_sp", 0);
            this.telemetry.addData("LF_p", 0);
            this.telemetry.addData("RF_sp", 0);
            this.telemetry.addData("RF_p", 0);

            // these are fine
            this.telemetry.addData("LB_v (sp)", 0);
            this.telemetry.addData("RB_v (sp)", 0);
            this.telemetry.addData("LF_v (sp)", 0);
            this.telemetry.addData("RF_v (sp)", 0);

            // these might not be fine
            this.telemetry.addData("LB Velocity", 0);
            this.telemetry.addData("RB Velocity", 0);
            this.telemetry.addData("LF Velocity", 0);
            this.telemetry.addData("RF Velocity", 0);

            this.telemetry.addData("LB Abs Error", 0);
            this.telemetry.addData("RB Abs Error", 0);
            this.telemetry.addData("LF Abs Error", 0);
            this.telemetry.addData("RF Abs Error", 0);

            telemetry.addData("LB Power", 0);
            telemetry.addData("RB Power", 0);
            telemetry.addData("LF Power", 0);
            telemetry.addData("RF Power", 0);

            this.telemetry.addData("HEADING", heading);

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
            throw new RuntimeException("driveByAngularEncoder(): Tried to run two drive actions at once");
        }

        // stop drivetrain at start
        if (!ignoreStartVelocity)
            stopController();

        // initialize variables
        double LB_start = getLBPosition(), RB_start = -getRBPosition(), LF_start = -getLFPosition(), RF_start = getRFPosition();
        double L = leftTicks, R = rightTicks;

        PIDController LB_PID = new PIDController(DrivePIDCoefficients.getDriveP(), DrivePIDCoefficients.getDriveI(), DrivePIDCoefficients.getDriveD(), LB_start);
        PIDController RB_PID = new PIDController(DrivePIDCoefficients.getDriveP(), DrivePIDCoefficients.getDriveI(), DrivePIDCoefficients.getDriveD(), RB_start);
        PIDController LF_PID = new PIDController(DrivePIDCoefficients.getDriveP(), DrivePIDCoefficients.getDriveI(), DrivePIDCoefficients.getDriveD(), LF_start);
        PIDController RF_PID = new PIDController(DrivePIDCoefficients.getDriveP(), DrivePIDCoefficients.getDriveI(), DrivePIDCoefficients.getDriveD(), RF_start);

        // calculate time needed to complete entire movement
        double totalDistance = Math.hypot(L,R);
        double maxVelocity = DrivePIDCoefficients.MAX_ATTAINABLE_VELOCITY;
        double[] travelTimes = calculateTravelTimes(totalDistance, maxVelocity, ignoreStartVelocity, ignoreEndVelocity);
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

            telemetry.addData("CURRENT MOVEMENT:", DriveSubsystem.currentMovement);
            telemetry.addData("MAX_VELOCITY", maxVelocity);

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
            double[] velocitySetpoints = calculateVelocitySetpoints(driveSpeed, theta, elapsedTime, travelTimes, maxVelocity, ignoreStartVelocity, ignoreEndVelocity);
            double LB_v = velocitySetpoints[0], RF_v = velocitySetpoints[0], RB_v = velocitySetpoints[1], LF_v = velocitySetpoints[1];

            // handle turn correction
            double turnVelocityGain = DrivePIDCoefficients.TURN_VELOCITY_GAIN;
            double turnPositionGain = DrivePIDCoefficients.TURN_POSITION_GAIN;
            double deltaTime = runtime.seconds() - elapsedTime;
            double currentHeading = getYaw();
            double thetaDiff = normalizeAngle(currentHeading - this.heading);
            telemetry.addData("HEADING", currentHeading);
            if (Math.abs(thetaDiff) < 15) {
                LB_v += turnVelocityGain * thetaDiff;
                RB_v -= turnVelocityGain * thetaDiff;
                LF_v += turnVelocityGain * thetaDiff;
                RF_v -= turnVelocityGain * thetaDiff;
                LBTurnPosDiff += turnPositionGain * thetaDiff * deltaTime;
                RBTurnPosDiff += turnPositionGain * thetaDiff * deltaTime;
                LFTurnPosDiff -= turnPositionGain * thetaDiff * deltaTime;
                RFTurnPosDiff -= turnPositionGain * thetaDiff * deltaTime;
            }

            // handle wheel positions
            double[] positionSetpoints = calculatePositionSetpoints(LB_start, RB_start, theta, elapsedTime, travelTimes, maxVelocity, ignoreStartVelocity, ignoreEndVelocity);
            double LB_sp = positionSetpoints[0], RF_sp = positionSetpoints[0], RB_sp = positionSetpoints[1], LF_sp = positionSetpoints[1];

            LB_PID.setSetPoint(LB_sp + LBTurnPosDiff);
            RB_PID.setSetPoint(RB_sp + RBTurnPosDiff);
            LF_PID.setSetPoint(LF_sp + LFTurnPosDiff);
            RF_PID.setSetPoint(RF_sp + RFTurnPosDiff);

            double LB_p = getLBPosition();
            double RB_p = -getRBPosition();
            double LF_p = getLFPosition();
            double RF_p = -getRFPosition();

            double velocityGain = DrivePIDCoefficients.VELOCITY_GAIN;
            double LB_C = LB_PID.update(LB_p) * velocityGain;
            double RB_C = RB_PID.update(RB_p) * velocityGain;
            double LF_C = LF_PID.update(LF_p) * velocityGain;
            double RF_C = RF_PID.update(RF_p) * velocityGain;

//            telemetry.addData("totalDistance", totalDistance);
//            telemetry.addData("totalTime", totalTime);
            telemetry.addData("LB_sp", LB_PID.getSetPoint());
            telemetry.addData("LB_p", LB_p);
            telemetry.addData("RB_sp", RB_PID.getSetPoint());
            telemetry.addData("RB_p", RB_p);
            telemetry.addData("LF_sp", LF_PID.getSetPoint());
            telemetry.addData("LF_p", LF_p);
            telemetry.addData("RF_sp", RF_PID.getSetPoint());
            telemetry.addData("RF_p", RF_p);

            // these are fine
            telemetry.addData("LB_C", LB_C);
            telemetry.addData("RB_C", RB_C);
            telemetry.addData("LF_C", LF_C);
            telemetry.addData("RF_C", RF_C);

            double L_AVG_C = (Math.abs(LB_C) + Math.abs(RF_C)) / 2.0;
            double R_AVG_C = (Math.abs(RB_C) + Math.abs(LF_C)) / 2.0;

            this.telemetry.addData("LB Abs Error", LB_PID.getAbsoluteDiff(LB_p));
            this.telemetry.addData("RB Abs Error", RB_PID.getAbsoluteDiff(RB_p));
            this.telemetry.addData("LF Abs Error", LF_PID.getAbsoluteDiff(LF_p));
            this.telemetry.addData("RF Abs Error", RF_PID.getAbsoluteDiff(RF_p));

            if (!(LB_PID.getAbsoluteDiff(LB_p) + RB_PID.getAbsoluteDiff(RB_p) + LF_PID.getAbsoluteDiff(LF_p) + RF_PID.getAbsoluteDiff(RF_p) < 4 * DrivePIDCoefficients.getErrorTolerance_p())) {
                LB_v += L_AVG_C * Math.signum(LB_C);
                RB_v += R_AVG_C * Math.signum(RB_C);
                LF_v += R_AVG_C * Math.signum(LF_C);
                RF_v += L_AVG_C * Math.signum(RF_C);
            }

            // these are fine
            telemetry.addData("LB_v (sp)", LB_v);
            telemetry.addData("RB_v (sp)", RB_v);
            telemetry.addData("LF_v (sp)", LF_v);
            telemetry.addData("RF_v (sp)", RF_v);

            // these might not be fine
            telemetry.addData("LB Velocity", getLBVelocity());
            telemetry.addData("RB Velocity", getRBVelocity());
            telemetry.addData("LF Velocity", getLFVelocity());
            telemetry.addData("RF Velocity", getRFVelocity());
            telemetry.addData("thetaDiff", thetaDiff);

            double theoreticalMaxVelocity = DrivePIDCoefficients.MAX_VELOCITY;
            double LF_power = (LF_v + LFTurnPosDiff) / theoreticalMaxVelocity;
            double RF_power = (RF_v + RFTurnPosDiff) / theoreticalMaxVelocity;
            double LB_power = (LB_v + LBTurnPosDiff) / theoreticalMaxVelocity;
            double RB_power = (RB_v + RBTurnPosDiff) / theoreticalMaxVelocity;

            telemetry.addData("LB Power", LB_power);
            telemetry.addData("RB Power", RB_power);
            telemetry.addData("LF Power", LF_power);
            telemetry.addData("RF Power", RF_power);

            // normalize velocities and drive with motor powers
            driveWithMotorPowers(
                    LF_power,
                    RF_power,
                    LB_power,
                    RB_power
                    );

            telemetry.update();

            // update elapsed time
            elapsedTime = runtime.seconds() - startTime;
        }

        // stop drivetrain at end
        if (!ignoreEndVelocity)
            stopController();
    }

    /**
     * Calculates the target positions for the leftBack and rightBack wheels in a movement at a certain time.
     */
    private static double[] calculatePositionSetpoints(double L_start, double R_start, double theta, double elapsedTime, double[] travelTimes, double maxVelocity, boolean ignoreStartVelocity, boolean ignoreEndVelocity) {

        // this method returns {L_sp, R_sp}

        double spread = DrivePIDCoefficients.VELOCITY_SPREAD_PROPORTION; // this is s
        double firstHalfTime = travelTimes[0]; // time needed to travel the first half of the distance
        double secondHalfTime = travelTimes[1]; // time needed to travel the second half of the distance
        double totalTime = firstHalfTime + secondHalfTime;

        double currentDistance = 0; // distance travelled to be calculated

        // five temporal waypoints:
        // W_0) 0
        // W_1) spread * firstHalfTime
        // W_2) firstHalfTime
        // W_3) firstHalfTime + (1 - spread) * secondHalfTime
        // W_4) firstHalfTime + secondHalfTime
        //
        //         V
        //         ^
        //     MAX -            ___________
        //         |          / .    .    . \
        //         |        /   .    .    .   \
        //         |      /     .    .    .     \
        //         |    /       .    .    .       \
        //         |  /         .    .    .         \
        //         |/           .    .    .           \
        //        -|------------|----|-----------------|-------> t
        //         W_0         W_1  W_2  W_3          W_4
        //
        double[] waypoints = new double[] {
                0,
                spread * firstHalfTime,
                firstHalfTime,
                firstHalfTime + (1 - spread) * secondHalfTime,
                firstHalfTime + secondHalfTime
        };
        double waypointTime;

        // calculate distance travelled
        // waypoint 1
        waypointTime = Math.max(elapsedTime - waypoints[0], 0); // elapsed non-negative time since waypoint 1
        if (elapsedTime >= waypoints[1]) // clip time to waypoint 1
            waypointTime = waypoints[1] - waypoints[0];
        // rectangular graph
        if (ignoreStartVelocity) {
            currentDistance += waypointTime * maxVelocity;
        }
        // trapezoidal graph
        else {
            double a = maxVelocity / waypoints[1]; // acceleration
            currentDistance += 0.5 * a * Math.pow(waypointTime, 2);
        }

        // waypoint 2
        waypointTime = Math.max(elapsedTime - waypoints[1], 0); // elapsed non-negative time since waypoint 2
        if (elapsedTime >= waypoints[2]) // clip time to waypoint 2
            waypointTime = waypoints[2] - waypoints[1];
        // rectangular graph
        currentDistance += waypointTime * maxVelocity;

        // waypoint 3
        waypointTime = Math.max(elapsedTime - waypoints[2], 0); // elapsed non-negative time since waypoint 3
        if (elapsedTime >= waypoints[3]) // clip time to waypoint 3
            waypointTime = waypoints[3] - waypoints[2];
        // rectangular graph
        currentDistance += waypointTime * maxVelocity;

        // waypoint 4
        waypointTime = Math.max(elapsedTime - waypoints[3], 0); // elapsed non-negative time since waypoint 4
        if (elapsedTime >= waypoints[4]) // clip time to waypoint 4
            waypointTime = waypoints[4] - waypoints[3];
        // rectangular graph
        if (ignoreEndVelocity) {
            currentDistance += waypointTime * maxVelocity;
        }
        // trapezoidal graph
        else {
            double a = maxVelocity / waypoints[1]; // acceleration
            currentDistance += 0.5 * (waypoints[4] - waypoints[3]) * (maxVelocity) - 0.5 * a * Math.pow(waypoints[4] - waypoints[3] - waypointTime, 2);
        }

        // calculate setpoints
        // LB
        double L_sp = L_start + currentDistance * Math.sin(theta - Math.PI / 4);
        // RB
        double R_sp = R_start + currentDistance * Math.sin(theta + Math.PI / 4);

        return new double[] { L_sp, R_sp };
    }

    /**
     * Calculates the target velocities for the leftBack and rightBack wheels in a movement at a certain time.
     */
    private static double[] calculateVelocitySetpoints(double driveSpeed, double theta, double elapsedTime, double[] travelTimes, double maxVelocity, boolean ignoreStartVelocity, boolean ignoreEndVelocity) {

        // this method returns {L_v, R_v}

        double spread = DrivePIDCoefficients.VELOCITY_SPREAD_PROPORTION; // this is s
        double firstHalfTime = travelTimes[0]; // time needed to travel the first half of the distance
        double secondHalfTime = travelTimes[1]; // time needed to travel the second half of the distance
        double totalTime = firstHalfTime + secondHalfTime;

        double LDir = Math.sin(theta - Math.PI / 4), RDir = Math.sin(theta + Math.PI / 4);
        double L_v, R_v; // LB and RB

        // in first half
        if (elapsedTime < firstHalfTime) {
            // rectangular graph
            if (ignoreStartVelocity) {
                L_v = (maxVelocity) * driveSpeed * LDir;
                R_v = (maxVelocity) * driveSpeed * RDir;
            }
            // trapezoidal graph
            else {
                double a = maxVelocity / (spread * firstHalfTime); // acceleration
                if (elapsedTime < spread * firstHalfTime) { // in slope
                    // y = at
                    L_v = (a * elapsedTime) * driveSpeed * LDir;
                    R_v = (a * elapsedTime) * driveSpeed * RDir;
                } else { // in plateau
                    L_v = (maxVelocity) * driveSpeed * LDir;
                    R_v = (maxVelocity) * driveSpeed * RDir;
                }
            }
        }
        // in second half
        else {
            // rectangular graph
            if (ignoreEndVelocity) {
                L_v = (maxVelocity) * driveSpeed * LDir;
                R_v = (maxVelocity) * driveSpeed * RDir;
            }
            // trapezoidal graph
            else {
                double a = maxVelocity / (spread * secondHalfTime); // acceleration
                if (elapsedTime < firstHalfTime + (1 - spread) * secondHalfTime) { // in plateau
                    L_v = (maxVelocity) * driveSpeed * LDir;
                    R_v = (maxVelocity) * driveSpeed * RDir;
                } else { // in slope
                    // y = a(T-t)
                    L_v = (a * (totalTime - elapsedTime)) * driveSpeed * LDir;
                    R_v = (a * (totalTime - elapsedTime)) * driveSpeed * RDir;
                }
            }
        }

        return new double[] { L_v, R_v };
    }

    /**
     * Calculates how long a drive movement will take.
     */
    private static double[] calculateTravelTimes(double distance, double maxVelocity, boolean ignoreStartVelocity, boolean ignoreEndVelocity) {

        // returns {firstHalfTime, secondHalfTime}
        // maxVelocity is V
        double spread = DrivePIDCoefficients.VELOCITY_SPREAD_PROPORTION; // this is s
        double totalDist = distance; // this is D
        double halfDist = totalDist / 2;

        // calculate time for each half of the distance travelled
        double firstHalfTime, // this is FH
                secondHalfTime; // this is SH

        // calculate time for the first half
        if (ignoreStartVelocity) {

            // the velocity vs time graph should look like this:
            //
            //         V
            //         ^
            //     MAX -__________________
            //         |                 |
            //         |                 |
            //         |                 |
            //         |       D/2       |
            //         |                 |
            //         |                 |
            //        -|-----------------|-----------> t
            //         0                FH
            //
            //  using area:
            //      FH * MAX = D/2
            //      FH = D/(2*MAX)

            firstHalfTime = totalDist / (2 * maxVelocity);
        }
        else {

            // the velocity vs time graph should look like this:
            //
            //         V
            //         ^
            //     MAX -            ______
            //         |          / .    |
            //         |        /   .    |
            //         |      /     .    |
            //         |    /     D/2    |
            //         |  /         .    |
            //         |/           .    |
            //        -|------------|----|-----------> t
            //         0           sFH  FH
            //
            //  using area:
            //      1/2 * [FH + (FH-sFH)] * MAX = D/2
            //      FH * (2-s) * MAX = D
            //                D
            //      FH = -----------
            //            (2-s)*MAX

            firstHalfTime = totalDist / ((2 - spread) * maxVelocity);
        }

        // calculate time for the second half
        if (ignoreEndVelocity) {

            // the velocity vs time graph should look like this:
            //
            //         V
            //         ^
            //     MAX -__________________
            //         |                 |
            //         |                 |
            //         |                 |
            //         |       D/2       |
            //         |                 |
            //         |                 |
            //        -|-----------------|-----------> t
            //         0                SH
            //
            //  using area:
            //      SH * MAX = D/2
            //      SH = D/(2*MAX)

            secondHalfTime = totalDist / (2 * maxVelocity);
        }
        else {

            // the velocity vs time graph should look like this:
            //
            //         V
            //         ^
            //     MAX -_____
            //         |    . \
            //         |    .   \
            //         |    .     \
            //         |    D/2     \
            //         |    .         \
            //         |    .           \
            //        -|----|------------|---> t
            //         0  (1-s)SH       SH
            //
            //  using area:
            //      1/2 * [SH + (1-s)SH] * MAX = D/2
            //      SH * (2-s) * MAX = D
            //                D
            //      SH = -----------
            //            (2-s)*MAX

            secondHalfTime = totalDist / ((2 - spread) * maxVelocity);
        }

        return new double[] { firstHalfTime, secondHalfTime };
    }

    /**
     * Use only for autonomous. Move a certain distance in ticks. Drive is robot centric.
     * @param turnSpeed      Positive rotation speed of the robot.
     * @param degrees          How many degrees the robot should turn
     */
    public void turnByIMU(double turnSpeed, double degrees) {
        // check for clashing actions
        if (DriveSubsystem.getIsBusy()) {
            throw new RuntimeException("turnByIMU(): Tried to run two drive actions at once");
        }

        // begin action
        double startAngle = getYaw();
        double setPoint = degrees;
        double cumulativeAngle = 0, previousAngle = heading;
        double K;

        PIDController PID = new PIDController(DrivePIDCoefficients.getTurnP(), DrivePIDCoefficients.getTurnI(), DrivePIDCoefficients.getTurnD(), setPoint);

        while (
                opMode.opModeIsActive() && !(
                        Math.abs(degrees - (cumulativeAngle + normalizeAngle(previousAngle-getYaw()))) < DrivePIDCoefficients.getErrorTolerance_degrees() &&
                                Math.abs(getLBVelocity()) + Math.abs(getRBVelocity()) + Math.abs(getLFVelocity()) + Math.abs(getRFVelocity()) < 4 * DrivePIDCoefficients.getErrorTolerance_v())
        ) {
            PID.setPID(DrivePIDCoefficients.getTurnP(), DrivePIDCoefficients.getTurnI(), DrivePIDCoefficients.getTurnD());

            double currentHeading = getYaw();

            cumulativeAngle += normalizeAngle(previousAngle-currentHeading);
            previousAngle = currentHeading;

            K = PID.update(cumulativeAngle);

            if (telemetry != null) {
                telemetry.addData("CURRENT MOVEMENT:", currentMovement);
                telemetry.addData("HEADING", currentHeading);
                telemetry.addData("Degrees Disp setpoint",setPoint);
                telemetry.addData("Degrees diff",setPoint-cumulativeAngle);
                telemetry.addData("Degrees cumulative",cumulativeAngle);
                telemetry.addData("K",K);
                telemetry.update();
            }

            driveRobotCentric(0, 0, turnSpeed * K);

            isBusy = true;
        }

        // brake
        stopController();
        heading += degrees;
        isBusy = false;
    }

    /**
     * Use only for autonomous. Follow the passed MovementSequence. Drive is robot centric.
     * @param movementSequence      The MovementSequence to be followed.
     */
    public void followMovementSequence(MovementSequence movementSequence) {

        leftBack.resetEncoder();
        rightBack.resetEncoder();
        leftFront.resetEncoder();
        rightFront.resetEncoder();

        ArrayDeque<Movement> movements = movementSequence.movements.clone();

        while (opMode.opModeIsActive() && !movements.isEmpty()) {
            ArrayList<Future<?>> threadStatus = new ArrayList<>();

            // fetch all linked movements
            boolean linked = true;
            while (!movements.isEmpty() && linked && opMode.opModeIsActive()) {
                Movement movement = movements.removeFirst();
                currentMovement = "["+movement.toString()+"]";
                telemetry.addData("CURRENT MOVEMENT:", currentMovement);
                telemetry.addData("IGNORE START:", movement.ignoreStartVelocity);
                telemetry.addData("IGNORE END:", movement.ignoreEndVelocity);
                telemetry.update();
                linked = movement.linkedToNext;
                MovementThread thread = new MovementThread(movement);
                Future<?> status = threadPool.submit(thread);
                threadStatus.add(status); // start the MovementThread
            }

            // wait until all linked movements are completed
            boolean running = true;
            while (running && opMode.opModeIsActive()) {
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
    }

    public double getYaw() {
        return (imu.getAngularOrientation().firstAngle * 180.0 / Math.PI + 180 + offsetAngle) % 360 - 180;
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
        runtime.reset();
        while (opMode.opModeIsActive() && (runtime.milliseconds() < ms));
    }

}