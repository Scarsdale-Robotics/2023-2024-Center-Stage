package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.SpeedCoefficients;
import org.firstinspires.ftc.teamcode.subsystems.movement.Movement;
import org.firstinspires.ftc.teamcode.subsystems.movement.MovementSequence;

import java.util.ArrayDeque;

public class DriveSubsystem extends SubsystemBase {
    private static final double TICKS_PER_INCH_FORWARD = 32.4;
    private static final double TICKS_PER_INCH_STRAFE = 62.5;
    private static final double TICKS_PER_DEGREE_TURN = 10.0;
    private static final double TICKS_PER_DEGREE_ARM = 35.0;
    final private ElapsedTime runtime = new ElapsedTime();
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
        double startEncoder = rightBack.getCurrentPosition(), targetEncoder = startEncoder + ticks;
        double kP = 1, d_error = 10.0, kV, delta; // kP will need tuning

        while (opMode.opModeIsActive() && Math.abs(rightBack.getCurrentPosition() - startEncoder) < ticks) {
            delta = Math.abs(targetEncoder - rightBack.getCurrentPosition() + d_error);
            kV = Math.min(1.0, delta * kP);
            driveRobotCentric(rightSpeed, forwardSpeed, turnSpeed);
        }

        controller.stop();
    }

    /**
     * Use only for autonomous. Follow the passed MovementSequence. Drive is robot centric.
     * @param movementSequence      The MovementSequence to be followed.
     */
    public void followMovementSequence(MovementSequence movementSequence) throws InterruptedException {
        double  POWER_FORWARD = SpeedCoefficients.getAutonomousForwardSpeed(),
                POWER_STRAFE = SpeedCoefficients.getAutonomousStrafeSpeed(),
                POWER_TURN = SpeedCoefficients.getAutonomousTurnSpeed(),
                POWER_ARM = SpeedCoefficients.getAutonomousArmSpeed();

        ArrayDeque<Movement> movements = movementSequence.movements.clone();

        while (opMode.opModeIsActive() && !movements.isEmpty()) {
            Movement movement = movements.pollFirst();
            Movement.MovementType type = movement.MOVEMENT_TYPE;

            driveByEncoder(
                    POWER_STRAFE * type.k_strafe,
                    POWER_FORWARD * type.k_forward,
                    POWER_TURN * type.k_turn,
                    movement.INCHES_STRAFE * TICKS_PER_INCH_STRAFE * Math.abs(type.k_strafe) +
                            movement.INCHES_FORWARD * TICKS_PER_INCH_FORWARD * Math.abs(type.k_forward) +
                            movement.DEGREES_TURN * TICKS_PER_DEGREE_TURN * Math.abs(type.k_turn)
            );

            inDep.raiseByEncoder(
                    POWER_ARM * type.k_elevation,
                    movement.DEGREES_ELEVATION * TICKS_PER_DEGREE_ARM * Math.abs(type.k_elevation)
            );

            if (type == Movement.MovementType.DELAY) {
                sleepFor(movement.WAIT);
            }

            if (type == Movement.MovementType.CLOSE_CLAW_RIGHT) {
                inDep.closeRightClaw();
            }
            if (type == Movement.MovementType.OPEN_CLAW_RIGHT) {
                inDep.openRightClaw();
            }

            if (type == Movement.MovementType.CLOSE_CLAW_LEFT) {
                inDep.closeLeftClaw();
            }
            if (type == Movement.MovementType.OPEN_CLAW_LEFT) {
                inDep.openLeftClaw();
            }
        }

        controller.stop();
    }

    /**
     * Smart sleep with opMode running check.
     * @param ms Timeout in milliseconds.
     */
    private void sleepFor(long ms) {
        runtime.reset();
        while (opMode.opModeIsActive() && (runtime.milliseconds() < ms));
    }

    /**
     * @return the current position of the robot's wheels in ticks.
     */
    public int getWheelPosition() {
        return rightBack.getCurrentPosition();
    }

    public void resetIMU() {
        imu.resetYaw();
    }

    private double getYaw() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }
}