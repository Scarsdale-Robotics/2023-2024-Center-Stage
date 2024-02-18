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
    private static PIDController leftFrontController = new PIDController(DrivePIDCoefficients.getVelocityP(), DrivePIDCoefficients.getVelocityI(), DrivePIDCoefficients.getVelocityD());
    private static PIDController rightFrontController = new PIDController(DrivePIDCoefficients.getVelocityP(), DrivePIDCoefficients.getVelocityI(), DrivePIDCoefficients.getVelocityD());
    private static PIDController leftBackController = new PIDController(DrivePIDCoefficients.getVelocityP(), DrivePIDCoefficients.getVelocityI(), DrivePIDCoefficients.getVelocityD());
    private static PIDController rightBackController = new PIDController(DrivePIDCoefficients.getVelocityP(), DrivePIDCoefficients.getVelocityI(), DrivePIDCoefficients.getVelocityD());
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
     * Use only for autonomous. Move a certain distance in ticks. Drive is robot centric.
     *
     * @param rightSpeed   How fast the robot should strafe to the right (negative values = strafe left).
     * @param forwardSpeed How fast the robot should move forward (negative values = move backwards).
     * @param ticks        How far the robot should move.
     */
    public void driveByRectilinearEncoder(double rightSpeed, double forwardSpeed, double ticks) {
        // check for clashing actions
        if (DriveSubsystem.getIsBusy()) {
            throw new RuntimeException("driveByRectilinearEncoder(): Tried to run two drive actions at once");
        }

        // begin action
        double startEncoder = getRBPosition();
        double setPoint = startEncoder + ticks;
        double K;

        PIDController PID = new PIDController(DrivePIDCoefficients.getDriveP(), DrivePIDCoefficients.getDriveI(), DrivePIDCoefficients.getDriveD(), setPoint);

        while (
                opMode.opModeIsActive() &&
                        Math.abs(setPoint- getRBPosition()) > DrivePIDCoefficients.getErrorTolerance_p() &&
                        Math.abs(getRBVelocity()) > DrivePIDCoefficients.getErrorTolerance_v()
        ) {
            K = PID.update(getRBPosition());
            driveRobotCentric(rightSpeed * K, forwardSpeed * K, 0);
            isBusy = true;
        }

        // brake
        controller.stop();
        isBusy = false;
    }

    /**
     * Use only for autonomous. Move a certain distance following two motion vectors for diagonal motor pairs. Drive is robot centric.
     * @param driveSpeed      Positive movement speed of the robot.
     * @param leftTicks     How many ticks the back left and front right wheels should be displaced by.
     * @param rightTicks      How many ticks the back right and front left wheels should be displaced by.
     * @param theta      The direction of movement in radians in [-π, π].
     */
    public void driveByAngularEncoder(double driveSpeed, double leftTicks, double rightTicks, double theta) {
        driveByAngularEncoder(driveSpeed, leftTicks, rightTicks, theta, false);
    }

    /**
     * Use only for autonomous. Move a certain distance following two motion vectors for diagonal motor pairs. Drive is robot centric.
     * @param driveSpeed      Positive movement speed of the robot.
     * @param leftTicks     How many ticks the back left and front right wheels should be displaced by.
     * @param rightTicks      How many ticks the back right and front left wheels should be displaced by.
     * @param theta      The direction of movement in radians in [-π, π].
     * @param ignoreVelocity    If the wheels should ignore fully stopping after reaching the setpoints.
     */
    public void driveByAngularEncoder(double driveSpeed, double leftTicks, double rightTicks, double theta, boolean ignoreVelocity) {
        // check for clashing actions
        if (DriveSubsystem.getIsBusy()) {
            throw new RuntimeException("driveByAngularEncoder(): Tried to run two drive actions at once");
        }

        // begin action
        double LB_start = getLBPosition(), RB_start = getRBPosition(), LF_start = getLFPosition(), RF_start = getRFPosition();
        double LD = leftTicks * Math.signum(Math.sin(theta - Math.PI / 4)), RD = rightTicks * Math.signum(Math.sin(theta + Math.PI / 4));
        double LB_sp = LB_start + LD,
                RB_sp = -RB_start + RD,
                LF_sp = -RB_start + RD,
                RF_sp = LB_start + LD;
        double L = leftTicks, R = rightTicks;

        PIDController LB_PID = new PIDController(DrivePIDCoefficients.getDriveP(), DrivePIDCoefficients.getDriveI(), DrivePIDCoefficients.getDriveD(), LB_sp);
        PIDController RB_PID = new PIDController(DrivePIDCoefficients.getDriveP(), DrivePIDCoefficients.getDriveI(), DrivePIDCoefficients.getDriveD(), RB_sp);
        PIDController LF_PID = new PIDController(DrivePIDCoefficients.getDriveP(), DrivePIDCoefficients.getDriveI(), DrivePIDCoefficients.getDriveD(), LF_sp);
        PIDController RF_PID = new PIDController(DrivePIDCoefficients.getDriveP(), DrivePIDCoefficients.getDriveI(), DrivePIDCoefficients.getDriveD(), RF_sp);

        boolean LB_atSetPoint = false, RB_atSetPoint = false, LF_atSetPoint = false, RF_atSetPoint = false;

        while ( // checking condition
                opMode.opModeIsActive() && !(
                        LB_atSetPoint &&
                        RB_atSetPoint &&
                        LF_atSetPoint &&
                        RF_atSetPoint)
        ) {
            // getting current positions
            double LB_p = getLBPosition();
            double RB_p = -getRBPosition();
            double LF_p = -getLFPosition();
            double RF_p = getRFPosition();

            LB_PID.setPID(DrivePIDCoefficients.getDriveP(), DrivePIDCoefficients.getDriveI(), DrivePIDCoefficients.getDriveD());
            RB_PID.setPID(DrivePIDCoefficients.getDriveP(), DrivePIDCoefficients.getDriveI(), DrivePIDCoefficients.getDriveD());
            LF_PID.setPID(DrivePIDCoefficients.getDriveP(), DrivePIDCoefficients.getDriveI(), DrivePIDCoefficients.getDriveD());
            RF_PID.setPID(DrivePIDCoefficients.getDriveP(), DrivePIDCoefficients.getDriveI(), DrivePIDCoefficients.getDriveD());

            double LB_K = LB_PID.update(LB_p);
            double RB_K = RB_PID.update(RB_p);
            double LF_K = LF_PID.update(LF_p);
            double RF_K = RF_PID.update(RF_p);

            double LDir = Math.abs(Math.sin(theta - Math.PI / 4)), RDir = Math.abs(Math.sin(theta + Math.PI / 4));
            double LB_v = driveSpeed * LDir * LB_K; // for bL
            double RB_v = driveSpeed * RDir * RB_K; // for bR
            double LF_v = driveSpeed * RDir * LF_K; // for fL
            double RF_v = driveSpeed * LDir * RF_K; // for fR

            if (telemetry != null) {
                telemetry.addData("CURRENT MOVEMENT:", currentMovement);
                telemetry.addData("LB setpoint",LB_sp);
                telemetry.addData("LB position",LB_p);
                telemetry.addData("LF setpoint",LF_sp);
                telemetry.addData("LF position",LF_p);
                telemetry.addData("RB setpoint",RB_sp);
                telemetry.addData("RB position",RB_p);
                telemetry.addData("RF setpoint",RF_sp);
                telemetry.addData("RF position",RF_p);
                telemetry.update();
            }

            double thetaDiff = 0.01 * normalizeAngle(getYaw()-this.heading); // originally factored into to each motor

            // setting the powers of the motors
//            double maxVelocity = DrivePIDCoefficients.MAX_VELOCITY;
//            driveWithMotorVelocities(
//                    maxVelocity * LF_v, // fL
//                    maxVelocity * RF_v, // fR
//                    maxVelocity * LB_v, // bL
//                    maxVelocity * RB_v  // bR
//            );

            driveWithMotorPowers(
                    LF_v, // fL
                    RF_v, // fR
                    LB_v, // bL
                    RB_v  // bR
            );

            double positionErrorTolerance = DrivePIDCoefficients.getErrorTolerance_p();
            LB_atSetPoint = LB_atSetPoint ||
                    (Math.abs(Math.abs(getLBPosition()-LB_start) - Math.abs(L)) < positionErrorTolerance); // position
            RB_atSetPoint = RB_atSetPoint ||
                    (Math.abs(Math.abs(getRBPosition()-RB_start) - Math.abs(R)) < positionErrorTolerance); // position
            LF_atSetPoint = LF_atSetPoint ||
                    (Math.abs(Math.abs(getLFPosition()-LF_start) - Math.abs(L)) < positionErrorTolerance); // position
            RF_atSetPoint = RF_atSetPoint ||
                    (Math.abs(Math.abs(getRFPosition()-RF_start) - Math.abs(R)) < positionErrorTolerance); // position

            isBusy = true;
        }

        // brake
        controller.stop();
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
        controller.stop();
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

        controller.stop();
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
        while (angle <= -180)
            angle += 360;
        while (angle > 180)
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
     */
    public void driveWithMotorVelocities(double leftFrontVelocity, double rightFrontVelocity, double leftBackVelocity, double rightBackVelocity) {

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

        double velocityToPower = 0.05;
        double LF_D = leftFrontController.update(getLFVelocity()) * velocityToPower;
        double RF_D = rightFrontController.update(-getRFVelocity()) * velocityToPower;
        double LB_D = leftBackController.update(getLBVelocity()) * velocityToPower;
        double RB_D = rightBackController.update(-getRBVelocity()) * velocityToPower;

        leftFrontPower += LF_D;
        rightFrontPower += RF_D;
        leftBackPower += LB_D;
        rightBackPower += RB_D;

        telemetry.addData("leftFront SP", leftFrontController.getSetPoint());
        telemetry.addData("leftFront Vel", getLFVelocity());
        telemetry.addData("----","");
        telemetry.addData("rightFront SP", rightFrontController.getSetPoint());
        telemetry.addData("rightFront Vel", getRFVelocity());
        telemetry.addData("-----","");
        telemetry.addData("leftBack SP", leftBackController.getSetPoint());
        telemetry.addData("leftBack Vel", getLBVelocity());
        telemetry.addData("------","");
        telemetry.addData("rightBack SP", rightBackController.getSetPoint());
        telemetry.addData("rightBack Vel", getRBVelocity());
        telemetry.update();

        driveWithMotorPowers(
                leftFrontPower,
                rightFrontPower,
                leftBackPower,
                rightBackPower
        );

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