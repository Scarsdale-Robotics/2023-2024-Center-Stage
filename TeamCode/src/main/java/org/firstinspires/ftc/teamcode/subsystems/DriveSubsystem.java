package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class DriveSubsystem extends SubsystemBase {
    private final MecanumDrive controller;

    private final IMU imu;

    public final Motor rightBack;

    private final LinearOpMode opMode;

    private int sideModifier = 1;

    public DriveSubsystem(Motor _leftFront, Motor _rightFront, Motor _leftBack, Motor _rightBack, IMU _imu, LinearOpMode _opMode) {
        controller = new MecanumDrive(
                _leftFront,
                _rightFront,
                _leftBack,
                _rightBack
        );

        this.imu = _imu;

        rightBack = _rightBack;

        opMode = _opMode;
    }

    /**
     * Drives robot centrically
     *
     * @param strafe  How much the robot should strafe.
     * @param forward How much the robot should move forwards and backwards.
     * @param turn    How much the robot should turn.
     */
    public void driveRobotCentric(double strafe, double forward, double turn) {
        controller.driveRobotCentric(strafe * sideModifier, forward, turn * sideModifier);
    }

    public void driveFieldCentric(double x, double y, double turn) {
        controller.driveFieldCentric(x, y, turn, getYaw());
    }

    public void drive(double power) {
        controller.driveRobotCentric(0, power, 0);

    }

    public void isRedSide(boolean value) {
        if (value) sideModifier = -1;
        else sideModifier = 1;
    }

    public void stop() {
        controller.stop();
    }

    public void leftStrafe(double power) {
        controller.driveRobotCentric(-power * sideModifier, 0, 0);
    }

    public void rightStrafe(double power) {
        controller.driveRobotCentric(power * sideModifier, 0, 0);
    }

    public double getYaw() {
        double angle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        opMode.telemetry.addData("Angle:", angle);
        return angle;
    }

    public void rotate(double power) {
        controller.driveRobotCentric(0, 0, power * sideModifier);
    }


    public void moveByEncoder(double strafePower, double forwardPower, double rotatePower, double steps, LiftSubsystem lift, LiftSubsystem.Level liftLevel) {
        if (lift != null) {
            lift.setLevel(liftLevel);
        }

        double startEncoder = rightBack.getCurrentPosition();

        while (opMode.opModeIsActive() && Math.abs(rightBack.getCurrentPosition() - startEncoder) < steps) {
            driveRobotCentric(strafePower, forwardPower, rotatePower);
            if (lift != null) lift.runLift();

            opMode.telemetry.addData("Delta Encoder: ", rightBack.getCurrentPosition() - startEncoder);
            opMode.telemetry.addData("Target Steps: ", steps);

            opMode.telemetry.addData("-------", null);

            opMode.telemetry.addData("Strafe Power: ", strafePower);
            opMode.telemetry.addData("Forward Power: ", forwardPower);
            opMode.telemetry.addData("Rotate Power: ", rotatePower);
            opMode.telemetry.addData("Lift Level: ", liftLevel);


            opMode.telemetry.update();
        }

        stop();
    }
}
