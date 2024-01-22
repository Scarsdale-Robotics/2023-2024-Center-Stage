package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="TEST Single Motor Speed")
public class SingleMotorSpeedTest extends LinearOpMode {
    public final String MOTOR_NAME = "arm1";

    @Override
    public void runOpMode() throws InterruptedException {
        Motor motor = new Motor(hardwareMap, MOTOR_NAME, Motor.GoBILDA.RPM_312);
        motor.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setRunMode(Motor.RunMode.VelocityControl);
        motor.resetEncoder();
        motor.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motor.setPositionTolerance(10);
//        motor.setPositionCoefficient(0.01);

        waitForStart();

        // CONTROLS:
        // move motor      --> triangle
        // inc. speed slow --> right trigger
        // dec. speed slow --> left trigger
        // inc. speed fast --> right trigger + (any) bumper
        // dec. speed fast --> right trigger + (any) bumper
        //
        // no speed change is allowed when motor is moving

        double speedChangeFactorSlow = 0.001;
        double speedChangeFactorFast = 0.01;
        double speed = speedChangeFactorSlow;
        while (opModeIsActive()) {
            if (gamepad1.triangle) {
                motor.motor.setPower(speed);
            } else {
                speed = Math.min(Math.max(speed + (gamepad1.right_trigger - gamepad1.left_trigger) * (((gamepad1.left_bumper || gamepad1.right_bumper) ? speedChangeFactorFast : speedChangeFactorSlow)), speedChangeFactorSlow), 1);
            }
            telemetry.addData("speed",  speed);
            telemetry.update();
        }
    }
}
