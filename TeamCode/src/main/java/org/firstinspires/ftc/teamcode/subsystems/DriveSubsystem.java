package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
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

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

public class DriveSubsystem extends SubsystemBase {

    private static volatile boolean isBusy;
    private static volatile ExecutorService threadPool;
    private final MecanumDrive controller;
    private final IMU imu;
    private final LinearOpMode opMode;
    private final Motor leftFront;
    private final Motor rightFront;
    private final Motor leftBack;
    private final Motor rightBack;
    private static double leftBackPower = 0;
    private static double rightBackPower = 0;
    private static double leftFrontPower = 0;
    private static double rightFrontPower = 0;
    private static PIDController leftFrontController  = new PIDController(DrivePIDCoefficients.getVelocityP(), DrivePIDCoefficients.getVelocityI(), DrivePIDCoefficients.getVelocityD());
    private static PIDController rightFrontController = new PIDController(DrivePIDCoefficients.getVelocityP(), DrivePIDCoefficients.getVelocityI(), DrivePIDCoefficients.getVelocityD());
    private static PIDController rightBackController  = new PIDController(DrivePIDCoefficients.getVelocityP(), DrivePIDCoefficients.getVelocityI(), DrivePIDCoefficients.getVelocityD());
    private static PIDController leftBackController   = new PIDController(DrivePIDCoefficients.getVelocityP(), DrivePIDCoefficients.getVelocityI(), DrivePIDCoefficients.getVelocityD());
    private Telemetry telemetry;
    public static volatile double heading=0D;
    private final ElapsedTime runtime;

    // for debugging
    public static String currentMovement="";

    public DriveSubsystem(Motor leftFront, Motor rightFront, Motor leftBack, Motor rightBack, IMU imu, LinearOpMode opMode) {
        this(leftFront, rightFront, leftBack, rightBack, imu, opMode, null);
    }

    public DriveSubsystem(Motor leftFront, Motor rightFront, Motor leftBack, Motor rightBack, IMU imu, LinearOpMode opMode, Telemetry telemetry) {
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
            this.telemetry.addData("L diff",0);
            this.telemetry.addData("R diff",0);

            this.telemetry.addData("Degrees setpoint",0);
            this.telemetry.addData("Degrees position",0);

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
        this.leftFrontPower = leftFrontPower;
        this.rightFrontPower = rightFrontPower;
        this.leftBackPower = leftBackPower;
        this.rightBackPower = rightBackPower;
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

        // prepare velocity PID controllers
        leftBackController.resetIntegral();
        rightBackController.resetIntegral();
        leftFrontController.resetIntegral();
        rightFrontController.resetIntegral();

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

        // calculate time needed to complete entire movement
        double totalDistance = Math.hypot(L,R);
        double[] travelTimes = calculateTravelTimes(totalDistance, ignoreStartVelocity, ignoreEndVelocity);
        double firstHalfTime = travelTimes[0]; // time needed to travel the first half of the distance
        double secondHalfTime = travelTimes[1]; // time needed to travel the second half of the distance
        double totalTime = firstHalfTime + secondHalfTime;

        double startTime = runtime.milliseconds() / 1000; // beginning time of the movement
        double elapsedTime = 0; // will act as the independent variable t for position & velocity calculations

        while (opMode.opModeIsActive() && elapsedTime < totalTime) {

            LB_PID.setPID(DrivePIDCoefficients.getDriveP(), DrivePIDCoefficients.getDriveI(), DrivePIDCoefficients.getDriveD());
            RB_PID.setPID(DrivePIDCoefficients.getDriveP(), DrivePIDCoefficients.getDriveI(), DrivePIDCoefficients.getDriveD());
            LF_PID.setPID(DrivePIDCoefficients.getDriveP(), DrivePIDCoefficients.getDriveI(), DrivePIDCoefficients.getDriveD());
            RF_PID.setPID(DrivePIDCoefficients.getDriveP(), DrivePIDCoefficients.getDriveI(), DrivePIDCoefficients.getDriveD());

            LB_PID.setErrorTolerance(DrivePIDCoefficients.getErrorTolerance_p());
            RB_PID.setErrorTolerance(DrivePIDCoefficients.getErrorTolerance_p());
            LF_PID.setErrorTolerance(DrivePIDCoefficients.getErrorTolerance_p());
            RF_PID.setErrorTolerance(DrivePIDCoefficients.getErrorTolerance_p());

            // handle wheel velocities
            double[] velocitySetpoints = calculateVelocitySetpoints(driveSpeed, theta, elapsedTime, travelTimes, ignoreStartVelocity, ignoreEndVelocity);
            double LB_v = velocitySetpoints[0], RF_v = velocitySetpoints[0], RB_v = velocitySetpoints[1], LF_v = velocitySetpoints[1];

            // handle wheel positions
            double[] positionSetpoints = calculatePositionSetpoints(LB_start, RB_start, driveSpeed, theta, elapsedTime, travelTimes, ignoreStartVelocity, ignoreEndVelocity);
            double LB_sp = positionSetpoints[0], RF_sp = positionSetpoints[0], RB_sp = positionSetpoints[1], LF_sp = positionSetpoints[1];

            LB_PID.setSetPoint(LB_sp);
            RB_PID.setSetPoint(RB_sp);
            LF_PID.setSetPoint(LF_sp);
            RF_PID.setSetPoint(RF_sp);

            double LB_p = getLBPosition();
            double RB_p = -getRBPosition();
            double LF_p = getLFPosition();
            double RF_p = -getRFPosition();

            double velocityGain = DrivePIDCoefficients.VELOCITY_GAIN;
            double LB_C = LB_PID.update(LB_p) * velocityGain;
            double RB_C = RB_PID.update(RB_p) * velocityGain;
            double LF_C = LF_PID.update(LF_p) * velocityGain;
            double RF_C = RF_PID.update(RF_p) * velocityGain;

            telemetry.addData("LB_sp", LB_sp);
            telemetry.addData("LB_p", LB_p);
            telemetry.addData("RB_sp", RB_sp);
            telemetry.addData("RB_p", RB_p);
            telemetry.addData("LF_sp", LF_sp);
            telemetry.addData("LF_p", LF_p);
            telemetry.addData("RF_sp", RF_sp);
            telemetry.addData("RF_p", RF_p);

            // these are fine
            telemetry.addData("LB_C", LB_C);
            telemetry.addData("RB_C", RB_C);
            telemetry.addData("LF_C", LF_C);
            telemetry.addData("RF_C", RF_C);

            if (!LB_PID.atSetPoint(LB_p))
                LB_v += LB_C;
            if (!RB_PID.atSetPoint(RB_p))
                RB_v += RB_C;
            if (!LF_PID.atSetPoint(LF_p))
                LF_v += LF_C;
            if (!RF_PID.atSetPoint(RF_p))
                RF_v += RF_C;

            // these are fine
            telemetry.addData("LB_v", LB_v);
            telemetry.addData("RB_v", RB_v);
            telemetry.addData("LF_v", LF_v);
            telemetry.addData("RF_v", RF_v);

            double thetaDiff = normalizeAngle(getYaw() - this.heading);
            if (Math.abs(thetaDiff) < 5) {
                LB_v += velocityGain * thetaDiff;
                RB_v -= velocityGain * thetaDiff;
                LF_v += velocityGain * thetaDiff;
                RF_v -= velocityGain * thetaDiff;
            }
            telemetry.addData("thetaDiff", thetaDiff);

            double maxVelocity = DrivePIDCoefficients.MAX_VELOCITY;
            // update motor velocities
            updateMotorVelocities(LF_v, RF_v, LB_v, RB_v);
//            driveWithMotorPowers(
//                    clipWheelVelocity(LF_v) / maxVelocity,
//                    clipWheelVelocity(RF_v) / maxVelocity,
//                    clipWheelVelocity(LB_v) / maxVelocity,
//                    clipWheelVelocity(RB_v) / maxVelocity
//                    );

            telemetry.update();

            // update elapsed time
            elapsedTime = runtime.milliseconds() / 1000 - startTime;
        }

        // stop drivetrain at end
        if (!ignoreEndVelocity)
            stopController();
    }

    /**
     * Calculates the target positions for the leftBack and rightBack wheels in a movement at a certain time.
     */
    private static double[] calculatePositionSetpoints(double L_start, double R_start, double driveSpeed, double theta, double elapsedTime, double[] travelTimes, boolean ignoreStartVelocity, boolean ignoreEndVelocity) {

        // this method returns {L_sp, R_sp}

        double maxVelocity = DrivePIDCoefficients.MAX_VELOCITY;
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
            currentDistance += waypointTime * maxVelocity * driveSpeed;
        }
        // trapezoidal graph
        else {
            double a = maxVelocity / waypoints[1]; // acceleration
            currentDistance += 0.5 * a * Math.pow(waypointTime, 2) * driveSpeed;
        }

        // waypoint 2
        waypointTime = Math.max(elapsedTime - waypoints[1], 0); // elapsed non-negative time since waypoint 2
        if (elapsedTime >= waypoints[2]) // clip time to waypoint 2
            waypointTime = waypoints[2] - waypoints[1];
        // rectangular graph
        currentDistance += waypointTime * maxVelocity * driveSpeed;

        // waypoint 3
        waypointTime = Math.max(elapsedTime - waypoints[2], 0); // elapsed non-negative time since waypoint 3
        if (elapsedTime >= waypoints[3]) // clip time to waypoint 3
            waypointTime = waypoints[3] - waypoints[2];
        // rectangular graph
        currentDistance += waypointTime * maxVelocity * driveSpeed;

        // waypoint 4
        waypointTime = Math.max(elapsedTime - waypoints[3], 0); // elapsed non-negative time since waypoint 4
        if (elapsedTime >= waypoints[4]) // clip time to waypoint 4
            waypointTime = waypoints[4] - waypoints[3];
        // rectangular graph
        if (ignoreEndVelocity) {
            currentDistance += waypointTime * maxVelocity * driveSpeed;
        }
        // trapezoidal graph
        else {
            double a = maxVelocity / waypoints[1]; // acceleration
            currentDistance += 0.5 * (waypoints[4] - waypoints[3]) * (maxVelocity * driveSpeed) - 0.5 * a * Math.pow(waypoints[4] - waypoints[3] - waypointTime, 2) * driveSpeed;
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
    private static double[] calculateVelocitySetpoints(double driveSpeed, double theta, double elapsedTime, double[] travelTimes, boolean ignoreStartVelocity, boolean ignoreEndVelocity) {

        // this method returns {L_v, R_v}

        double maxVelocity = DrivePIDCoefficients.MAX_VELOCITY;
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
    private static double[] calculateTravelTimes(double distance, boolean ignoreStartVelocity, boolean ignoreEndVelocity) {

        // returns {firstHalfTime, secondHalfTime}

        double maxVelocity = DrivePIDCoefficients.MAX_VELOCITY; // this is V
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
        double cumulativeAngle = 0, previousAngle = getYaw();
        double K;

        PIDController PID = new PIDController(DrivePIDCoefficients.getTurnP(), DrivePIDCoefficients.getTurnI(), DrivePIDCoefficients.getTurnD(), setPoint);

        while (
                opMode.opModeIsActive() && !(
                        Math.abs(degrees - (cumulativeAngle + normalizeAngle(previousAngle-getYaw()))) < DrivePIDCoefficients.getErrorTolerance_degrees() &&
                                Math.abs(getRBVelocity()) < DrivePIDCoefficients.getErrorTolerance_v())
        ) {
            PID.setPID(DrivePIDCoefficients.getTurnP(), DrivePIDCoefficients.getTurnI(), DrivePIDCoefficients.getTurnD());

            double currentYaw = getYaw();

            cumulativeAngle += normalizeAngle(previousAngle-currentYaw);
            previousAngle = currentYaw;

            K = PID.update(cumulativeAngle);

            if (telemetry != null) {
                telemetry.addData("CURRENT MOVEMENT:", currentMovement);
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
        while (getYaw() == 0D && heading == 0D && opMode.opModeIsActive()) { // wait until imu reads heading correctly
            heading = getYaw();
        }
        heading = getYaw();
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
            while (!movements.isEmpty() && linked && opMode.opModeIsActive()) {
                Movement movement = movements.removeFirst();
                currentMovement = "["+movement.toString()+"]";
                telemetry.addData("CURRENT MOVEMENT:", currentMovement);
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
    public double getRBVelocity() { return rightBack.getCorrectedVelocity(); }

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
        leftBackPower = 0;
        rightBackPower = 0;
        leftFrontPower = 0;
        rightFrontPower = 0;
    }

    public IMU getIMU() {
        return imu;
    }

    private void resetIMU() {
        imu.resetYaw();
    }

    public double getYaw() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
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
     * Drives the motors using PID control with the specified motor velocities.
     *
     * @param leftFrontVelocity     the velocity of the leftFront motor in encoder ticks/second.
     * @param rightFrontVelocity    the velocity of the rightFront motor in encoder ticks/second.
     * @param leftBackVelocity     the velocity of the leftBack motor in encoder ticks/second.
     * @param rightBackVelocity     the velocity of the rightBack motor in encoder ticks/second.
     * @return  whether all of the wheels have reached the target velocity.
     */
    public boolean updateMotorVelocities(double leftFrontVelocity, double rightFrontVelocity, double leftBackVelocity, double rightBackVelocity) {

        leftFrontVelocity = clipWheelVelocity(leftFrontVelocity);
        rightFrontVelocity = clipWheelVelocity(rightFrontVelocity);
        leftBackVelocity = clipWheelVelocity(leftBackVelocity);
        rightBackVelocity = clipWheelVelocity(rightBackVelocity);

        leftFrontController.setErrorTolerance(DrivePIDCoefficients.getErrorTolerance_v());
        rightFrontController.setErrorTolerance(DrivePIDCoefficients.getErrorTolerance_v());
        leftBackController.setErrorTolerance(DrivePIDCoefficients.getErrorTolerance_v());
        rightBackController.setErrorTolerance(DrivePIDCoefficients.getErrorTolerance_v());

        leftFrontController.setPID(DrivePIDCoefficients.getVelocityP(), DrivePIDCoefficients.getVelocityI(), DrivePIDCoefficients.getVelocityD());
        rightFrontController.setPID(DrivePIDCoefficients.getVelocityP(), DrivePIDCoefficients.getVelocityI(), DrivePIDCoefficients.getVelocityD());
        leftBackController.setPID(DrivePIDCoefficients.getVelocityP(), DrivePIDCoefficients.getVelocityI(), DrivePIDCoefficients.getVelocityD());
        rightBackController.setPID(DrivePIDCoefficients.getVelocityP(), DrivePIDCoefficients.getVelocityI(), DrivePIDCoefficients.getVelocityD());

        leftFrontController.setSetPoint(leftFrontVelocity);
        rightFrontController.setSetPoint(rightFrontVelocity);
        leftBackController.setSetPoint(leftBackVelocity);
        rightBackController.setSetPoint(rightBackVelocity);

        double maxVelocity = DrivePIDCoefficients.MAX_VELOCITY;

        double LF_c_v = Double.MAX_VALUE; // get corrected LF_v
        while (opMode.opModeIsActive() && Math.abs(LF_c_v) > maxVelocity)
            LF_c_v = getLFVelocity();

        double RF_c_v = Double.MAX_VALUE; // get corrected RF_v
        while (opMode.opModeIsActive() && Math.abs(RF_c_v) > maxVelocity)
            RF_c_v = -getRFVelocity();

        double LB_c_v = Double.MAX_VALUE; // get corrected LB_v
        while (opMode.opModeIsActive() && Math.abs(LB_c_v) > maxVelocity)
            LB_c_v = getLBVelocity();

        double RB_c_v = Double.MAX_VALUE; // get corrected RB_v
        while (opMode.opModeIsActive() && Math.abs(RB_c_v) > maxVelocity)
            RB_c_v = -getRBVelocity();

        double powerGain = DrivePIDCoefficients.POWER_GAIN;
        double LF_D = leftFrontController.update(LF_c_v) * powerGain;
        double RF_D = rightFrontController.update(RF_c_v) * powerGain;
        double LB_D = leftBackController.update(LB_c_v) * powerGain;
        double RB_D = rightBackController.update(RB_c_v) * powerGain;

        // these are not fine
        telemetry.addData("LF_D", LF_D);
        telemetry.addData("RF_D", RF_D);
        telemetry.addData("LB_D", LB_D);
        telemetry.addData("RB_D", RB_D);

        if (!leftFrontController.atSetPoint(LF_c_v))
            leftFrontPower += LF_D;
        if (!rightFrontController.atSetPoint(RF_c_v))
            rightFrontPower += RF_D;
        if (!leftBackController.atSetPoint(LB_c_v))
            leftBackPower += LB_D;
        if (!rightBackController.atSetPoint(RB_c_v))
            rightBackPower += RB_D;

        driveWithMotorPowers(
                leftFrontPower,
                rightFrontPower,
                leftBackPower,
                rightBackPower
        );

        boolean atSetPoint = leftFrontController.atSetPoint(LF_c_v) &&
                rightFrontController.atSetPoint(RF_c_v) &&
                leftBackController.atSetPoint(LB_c_v) &&
                rightBackController.atSetPoint(RB_c_v);

        // power is not fine

        telemetry.addData("leftFront SP", leftFrontController.getSetPoint());
        telemetry.addData("LF_c_v", LF_c_v);
        telemetry.addData("leftFrontPower", leftFrontPower);
        telemetry.addData("----","");
        telemetry.addData("rightFront SP", rightFrontController.getSetPoint());
        telemetry.addData("RF_c_v", RF_c_v);
        telemetry.addData("rightFrontPower", rightFrontPower);
        telemetry.addData("-----","");
        telemetry.addData("leftBack SP", leftBackController.getSetPoint());
        telemetry.addData("LB_c_v", LB_c_v);
        telemetry.addData("leftBackPower", leftBackPower);
        telemetry.addData("------","");
        telemetry.addData("rightBack SP", rightBackController.getSetPoint());
        telemetry.addData("RB_c_v", RB_c_v);
        telemetry.addData("rightBackPower", rightBackPower);
        telemetry.addData("-------","");
        telemetry.addData("atSetPoint", atSetPoint);

        return atSetPoint;
    }

    /**
     * Smart sleep with opMode running check.
     * @param ms Timeout in milliseconds.
     */
    private void sleepFor(long ms) {
        runtime.reset();
        while (opMode.opModeIsActive() && (runtime.milliseconds() < ms));
    }

    private double clipWheelVelocity(double velocity) {
        if (velocity > DrivePIDCoefficients.MAX_VELOCITY)
            return DrivePIDCoefficients.MAX_VELOCITY;
        else if (velocity < -DrivePIDCoefficients.MAX_VELOCITY)
            return -DrivePIDCoefficients.MAX_VELOCITY;
        else return velocity;
    }

}