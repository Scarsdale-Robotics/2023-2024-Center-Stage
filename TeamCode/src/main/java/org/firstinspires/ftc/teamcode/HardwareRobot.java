package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

public class HardwareRobot {
    public final Motor leftFront;
    public final Motor rightFront;
    public final Motor leftBack;
    public final Motor rightBack;
    public final Motor arm1;
    public final Motor arm2;
    public final Servo elbow;
    public final Servo leftClaw;
    public final Servo rightClaw;
    public final Servo wrist;

    public final IMU imu;
//    public final OpenCvCamera frontCam;
    public final WebcamName frontCamName;
//    public final OpenCvCamera backCam;
    public final WebcamName backCamName;
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

        leftFront.setInverted(false);
        rightFront.setInverted(false);
        leftBack.setInverted(false);
        rightBack.setInverted(false);

        leftFront.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        leftFront.setPositionTolerance(10);
        rightFront.setPositionTolerance(10);
        leftBack.setPositionTolerance(10);
        rightBack.setPositionTolerance(10);

        arm1 = new Motor(hardwareMap, "arm1", Motor.GoBILDA.RPM_312);
        arm1.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm1.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm1.setRunMode(Motor.RunMode.VelocityControl);
        arm1.resetEncoder();
        arm1.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm1.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        arm1.setPositionTolerance(10);
        arm1.setPositionCoefficient(0.01);

        arm2 = new Motor(hardwareMap, "arm2", Motor.GoBILDA.RPM_312);
        arm2.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm2.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm2.setRunMode(Motor.RunMode.VelocityControl);
        arm2.resetEncoder();
        arm2.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm2.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        arm2.setPositionTolerance(10);
        arm2.setPositionCoefficient(0.01);

        leftClaw = hardwareMap.servo.get("leftClaw");
        rightClaw = hardwareMap.servo.get("rightClaw");
        wrist = hardwareMap.servo.get("wrist");
        elbow = hardwareMap.servo.get("elbow");

        leftClaw.scaleRange(0, 1);
        rightClaw.scaleRange(0, 1);
        wrist.scaleRange(0, 1);
        elbow.scaleRange(0, 1);

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();


        backCamName = hardwareMap.get(WebcamName.class, "Webcam Back");
//        backCam = OpenCvCameraFactory.getInstance().createWebcam(backCamName);

        frontCamName = hardwareMap.get(WebcamName.class, "Webcam Front");
//        frontCam = OpenCvCameraFactory.getInstance().createWebcam(frontCamName);
    }

    public double getYaw() {
        double angle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        return angle;
    }
}