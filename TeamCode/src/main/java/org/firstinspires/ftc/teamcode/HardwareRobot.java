package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvInternalCamera;

public class HardwareRobot {
    public final Motor leftFront;
    public final Motor rightFront;
    public final Motor leftBack;
    public final Motor rightBack;

    public final Motor lift;

    public final IMU imu;

    public final Servo claw;

    public final OpenCvCamera camera;

    public HardwareRobot(HardwareMap hardwareMap) {
        leftFront = new Motor(hardwareMap, "leftFront", Motor.GoBILDA.RPM_312);
        rightFront = new Motor(hardwareMap, "rightFront", Motor.GoBILDA.RPM_312);
        leftBack = new Motor(hardwareMap, "leftBack", Motor.GoBILDA.RPM_312);
        rightBack = new Motor(hardwareMap, "rightBack", Motor.GoBILDA.RPM_312);

        leftFront.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setRunMode(Motor.RunMode.VelocityControl);
        rightFront.setRunMode(Motor.RunMode.VelocityControl);
        leftBack.setRunMode(Motor.RunMode.VelocityControl);
        rightBack.setRunMode(Motor.RunMode.VelocityControl);

        leftFront.setInverted(true);
        leftBack.setInverted(true);
        rightBack.setInverted(true);
        rightFront.setInverted(true);

        lift = new Motor(hardwareMap, "lift", Motor.GoBILDA.RPM_312);
        lift.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setRunMode(Motor.RunMode.PositionControl);
        lift.setInverted(true);
        lift.resetEncoder();
        lift.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        lift.setPositionTolerance(10);
        lift.setPositionCoefficient(0.01);

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.FORWARD;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.LEFT;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        claw = hardwareMap.servo.get("claw");
        claw.scaleRange(0, 1);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

    }

    public double getYaw() {
        double angle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        return angle;
    }
}