package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.movement.MovementThread;
import org.firstinspires.ftc.teamcode.util.SpeedCoefficients;
import org.firstinspires.ftc.teamcode.subsystems.movement.Movement;
import org.firstinspires.ftc.teamcode.subsystems.movement.MovementSequence;
import org.firstinspires.ftc.teamcode.util.PIDController;

import java.util.ArrayDeque;
import java.util.ArrayList;

public class DriveSubsystem extends SubsystemBase {
    private static final double Kp = 0.01;
    private static final double Ki = 0;
    private static final double Kd = 0;
    private static final double errorTolerance_p = 5.0;
    private static final double errorTolerance_v = 0.05;

    private boolean isBusy;
    private MecanumDrive controller;
    private IMU imu;
    private LinearOpMode opMode;
    private Motor rightBack;
    private InDepSubsystem inDep;

    public DriveSubsystem(Motor leftFront, Motor rightFront, Motor leftBack, Motor rightBack, IMU imu, InDepSubsystem inDep, LinearOpMode opMode) {
        this.rightBack = rightBack;
        controller = new MecanumDrive(
                leftFront,
                rightFront,
                leftBack,
                rightBack
        );
        this.imu = imu;
        this.opMode = opMode;
        this.inDep = inDep;
        isBusy = false;
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
     * @param rightSpeed      How fast the robot should strafe to the right (negative values = strafe left).
     * @param forwardSpeed  How fast the robot should move forward (negative values = strafe backwards).
     * @param turnSpeed      How fast the robot should turn (clockwise?).
     * @param ticks          How far the robot should move.
     */
    public void driveByEncoder(double rightSpeed, double forwardSpeed, double turnSpeed, double ticks) {
        // check for clashing actions
        if (isBusy) {
            throw new RuntimeException("Tried to run two arm actions at once (isBusy = true)");
        }

        // begin action
        double startEncoder = rightBack.getCurrentPosition();
        double setPoint = startEncoder + ticks;
        double pidMultiplier;
        PIDController pidController = new PIDController(Kp, Ki, Kd, setPoint);

        while (
                opMode.opModeIsActive() &&
                Math.abs(setPoint-getWheelPosition()) > errorTolerance_p &&
                Math.abs(getWheelVelocity()) > errorTolerance_v
        ) {
            pidMultiplier = pidController.update(rightBack.getCurrentPosition());
            driveRobotCentric(rightSpeed * pidMultiplier, forwardSpeed * pidMultiplier, turnSpeed * pidMultiplier);
            isBusy = true;
        }

        controller.stop();
        isBusy = false;
    }

    /**
     * Use only for autonomous. Follow the passed MovementSequence. Drive is robot centric.
     * @param movementSequence      The MovementSequence to be followed.
     */
    public void followMovementSequence(MovementSequence movementSequence) {


        ArrayDeque<Movement> movements = movementSequence.movements.clone();

        while (opMode.opModeIsActive() && !movements.isEmpty()) {
            ArrayList<Thread> threads = new ArrayList<>();

            // fetch all linked movements
            boolean linked = true;
            while (!movements.isEmpty() && linked) {
                Movement movement = movements.pollFirst();
                MovementThread thread = new MovementThread(movement, this, inDep, opMode);
                linked = movement.linkedToNext;
                threads.add(new Thread(thread));
            }

            // start all movements
            for (Thread thread : threads) thread.start();

            // wait until all movements are completed
            boolean running = true;
            while (opMode.opModeIsActive() && running) {
                running = false;
                for (Thread thread : threads) running = thread.isAlive() || running; // running is only false if all threads are inactive
            }

        }

        controller.stop();
    }

    /**
     * @return the current position of the robot's wheels in ticks.
     */
    public int getWheelPosition() {
        return rightBack.getCurrentPosition();
    }

    /**
     * @return the current power of the robot's wheels.
     */
    public double getWheelVelocity() {
        return rightBack.getCorrectedVelocity();
    }

    /**
     * @return whether or not the drivetrain is in an action.
     */
    public boolean isBusy() {
        return isBusy;
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

    private double getYaw() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }
}