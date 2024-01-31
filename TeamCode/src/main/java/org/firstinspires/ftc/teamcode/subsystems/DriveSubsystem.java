package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;

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
    private Telemetry telemetry;
    public double theta;

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
        this.imu.resetYaw();
        this.theta = getYaw();
        isBusy = false;
        threadPool = Executors.newCachedThreadPool();

        if (this.telemetry != null) {
            this.telemetry.addData("L diff",0);
            this.telemetry.addData("R diff",0);

            this.telemetry.addData("Degrees setpoint",0);
            this.telemetry.addData("Degrees position",0);

            this.telemetry.update();
        }
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
     * @param frontLeftSpeed     the speed of the front left motor
     * @param frontRightSpeed     the speed of the front right motor
     * @param backLeftSpeed     the speed of the back left motor
     * @param backRightSpeed     the speed of the back right motor
     */
    public void driveWithMotorPowers(double frontLeftSpeed, double frontRightSpeed, double backLeftSpeed, double backRightSpeed) {
        controller.driveWithMotorPowers(frontLeftSpeed, frontRightSpeed, backLeftSpeed, backRightSpeed);
    }

    /**
     * Drives with directions based on driver pov.
     *
     * @param right     How much right the robot should strafe (negative values = strafe left).
     * @param forward   How much forward the robot should move (negative values = move backwards).
     * @param turn      How much the robot should turn.
     */
    public void driveFieldCentric(double right, double forward, double turn) {
        controller.driveFieldCentric(right, forward, turn, getYaw());
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
        double startEncoder = getRightWheelPosition();
        double setPoint = startEncoder + ticks;
        double K;

        PIDController PID = new PIDController(DrivePIDCoefficients.getDriveP(), DrivePIDCoefficients.getDriveI(), DrivePIDCoefficients.getDriveD(), setPoint);

        while (
                opMode.opModeIsActive() &&
                        Math.abs(setPoint- getRightWheelPosition()) > DrivePIDCoefficients.getErrorTolerance_p() &&
                        Math.abs(getRightWheelVelocity()) > DrivePIDCoefficients.getErrorTolerance_v()
        ) {
            K = PID.update(getRightWheelPosition());
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
        // check for clashing actions
        if (DriveSubsystem.getIsBusy()) {
            throw new RuntimeException("driveByAngularEncoder(): Tried to run two drive actions at once");
        }

        // begin action
        double L_start = getLeftWheelPosition(), R_start = getRightWheelPosition();
        double L_sp = leftBack.getCurrentPosition() + leftTicks * Math.signum(Math.sin(theta - Math.PI / 4)),
                R_sp = -rightBack.getCurrentPosition() + rightTicks * Math.signum(Math.sin(theta + Math.PI / 4));
        double L = leftTicks, R = rightTicks;

        PIDController L_PID = new PIDController(DrivePIDCoefficients.getDriveP(), DrivePIDCoefficients.getDriveI(), DrivePIDCoefficients.getDriveD(), L_sp);
        PIDController R_PID = new PIDController(DrivePIDCoefficients.getDriveP(), DrivePIDCoefficients.getDriveI(), DrivePIDCoefficients.getDriveD(), R_sp);

        boolean L_atSetPoint = false, R_atSetPoint = false;

        while ( // checking condition
                opMode.opModeIsActive() && !(
                        L_atSetPoint &&
                                R_atSetPoint)
        ) {
            // getting L and R
            double L_p = leftBack.getCurrentPosition();
            double R_p = -rightBack.getCurrentPosition();

            L_PID.setPID(DrivePIDCoefficients.getDriveP(), DrivePIDCoefficients.getDriveI(), DrivePIDCoefficients.getDriveD());
            R_PID.setPID(DrivePIDCoefficients.getDriveP(), DrivePIDCoefficients.getDriveI(), DrivePIDCoefficients.getDriveD());

            double L_K = L_PID.update(L_p);
            double R_K = R_PID.update(R_p);
            double universal_K;
            if (Math.abs(L_K) > Math.abs(R_K)) {
                if (!L_atSetPoint) {
                    universal_K = L_K;
                } else {
                    universal_K = R_K;
                }
            } else {
                if (!R_atSetPoint) {
                    universal_K = R_K;
                } else {
                    universal_K = L_K;
                }
            }
            universal_K = Math.abs(universal_K);

            double L_v = driveSpeed * Math.abs(Math.sin(theta - Math.PI / 4)) * universal_K * Math.signum(L_K); // for bL and fR
            double R_v = driveSpeed * Math.abs(Math.sin(theta + Math.PI / 4)) * universal_K * Math.signum(R_K); // for bR and fL

            if (telemetry != null) {
                telemetry.addData("L diff",Math.abs(Math.abs(getLeftWheelPosition()-L_start) - Math.abs(L)));
                telemetry.addData("L setpoint",L_sp);
                telemetry.addData("L position",L_p);
                telemetry.addData("R diff",Math.abs(Math.abs(getRightWheelPosition()-R_start) - Math.abs(R)));
                telemetry.addData("R setpoint",R_sp);
                telemetry.addData("R position",R_p);
                telemetry.update();
            }

            double thetaDiff = 0.01 * (getYaw()-this.theta);
            // setting the powers of the motors
            driveWithMotorPowers(
                    R_v + thetaDiff, // fL
                    L_v - thetaDiff, // fR
                    L_v + thetaDiff, // bL
                    R_v - thetaDiff  // bR
            );

            L_atSetPoint = L_atSetPoint ||
                    (Math.abs(Math.abs(getLeftWheelPosition()-L_start) - Math.abs(L)) < DrivePIDCoefficients.getErrorTolerance_p() &&
                            Math.abs(getLeftWheelVelocity()) < DrivePIDCoefficients.getErrorTolerance_v());
            R_atSetPoint = R_atSetPoint ||
                    (Math.abs(Math.abs(getRightWheelPosition()-R_start) - Math.abs(R)) < DrivePIDCoefficients.getErrorTolerance_p() &&
                            Math.abs(getRightWheelVelocity()) < DrivePIDCoefficients.getErrorTolerance_v());

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
                                Math.abs(getRightWheelVelocity()) < DrivePIDCoefficients.getErrorTolerance_v())
        ) {
            PID.setPID(DrivePIDCoefficients.getTurnP(), DrivePIDCoefficients.getTurnI(), DrivePIDCoefficients.getTurnD());

            double currentYaw = getYaw();

            cumulativeAngle += normalizeAngle(previousAngle-currentYaw);
            previousAngle = currentYaw;

            K = PID.update(cumulativeAngle);

            if (telemetry != null) {
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
        theta = getYaw();
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
                linked = movement.linkedToNext;
                MovementThread thread = new MovementThread(movement);
                Future<?> status = threadPool.submit(thread);
                threadStatus.add(status); // start the MovementThread
            }

            // wait until all linked movements are completed
            boolean running = true;
            while (opMode.opModeIsActive() && running && opMode.opModeIsActive()) {
                running = false;
                for (Future<?> status : threadStatus)
                    running = !status.isDone() || running; // running is only false if all threads are inactive
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
    public int getLeftWheelPosition() {
        return leftBack.getCurrentPosition();
    }

    /**
     * @return the current position of the robot's back right wheel in ticks.
     */
    public int getRightWheelPosition() {
        return rightBack.getCurrentPosition();
    }

    /**
     * @return the current power of the robot's back left wheel.
     */
    public double getLeftWheelVelocity() {
        return leftBack.getCorrectedVelocity();
    }

    /**
     * @return the current power of the robot's back right wheel.
     */
    public double getRightWheelVelocity() {
        return rightBack.getCorrectedVelocity();
    }

    /**
     * Stop the motors.
     */
    public void stopController() {
        controller.stop();
    }

    public void resetIMU() {
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
}