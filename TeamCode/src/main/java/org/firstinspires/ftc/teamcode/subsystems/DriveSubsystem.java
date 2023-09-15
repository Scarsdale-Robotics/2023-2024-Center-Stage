package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class DriveSubsystem extends SubsystemBase {
    private MecanumDrive controller;
    private IMU imu;
    private LinearOpMode opMode;
    private Motor rightBack;

    public DriveSubsystem(Motor leftFront, Motor rightFront, Motor leftBack, Motor rightBack, IMU imu, LinearOpMode opMode) {
        this.rightBack = rightBack;
        controller = new MecanumDrive(
                leftFront,
                rightFront,
                leftBack,
                rightBack
        );
        this.imu = imu;
        this.opMode = opMode;
    }

    public void drive(double left, double forward, double turn, boolean fieldCentric) {
        if (fieldCentric)
            driveFieldCentric(left, forward, turn);
        else
            driveRobotCentric(left, forward, turn);
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
     * Use only for autonomous. Move a certain distance in steps. Drive is field centric.
     * @param rightSpeed    How fast the robot should strafe to the right (negative values = strafe left).
     * @param forwardSpeed  How fast the robot should move forward (negative values = strafe backwards).
     * @param turnSpeed     How fast the robot should turn.
     * @param steps         How far the robot should move.
     */
    public void driveByEncoder(double rightSpeed, double forwardSpeed, double turnSpeed, double steps) {
        double startEncoder = rightBack.getCurrentPosition();

        while (opMode.opModeIsActive() && Math.abs(rightBack.getCurrentPosition() - startEncoder) < steps) {
            driveRobotCentric(rightSpeed, forwardSpeed, turnSpeed);
        }

        controller.stop();
    }

    public void resetIMU() {
        imu.resetYaw();
    }

    private double getYaw() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }
}
