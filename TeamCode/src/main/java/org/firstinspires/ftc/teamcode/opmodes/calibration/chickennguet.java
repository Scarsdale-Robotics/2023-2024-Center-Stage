package org.firstinspires.ftc.teamcode.opmodes.calibration;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subsystems.InDepSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RobotSystem;

@TeleOp(name = "chicken nguett")
public class chickennguet extends LinearOpMode {

    private double speed = 0.1;
    private Motor arm1;
    private Motor arm2;
    @Override
    public void runOpMode() throws InterruptedException {
        arm1 = new Motor(hardwareMap, "arm1", Motor.GoBILDA.RPM_312);
        arm1.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm1.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm1.setRunMode(Motor.RunMode.VelocityControl);
        arm1.resetEncoder();
        arm1.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm1.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        arm1.setPositionTolerance(10);
        arm1.setPositionCoefficient(0.01);
        arm1.setInverted(true);

        arm2 = new Motor(hardwareMap, "arm2", Motor.GoBILDA.RPM_312);
        arm2.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm2.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm2.setRunMode(Motor.RunMode.VelocityControl);
        arm2.resetEncoder();
        arm2.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm2.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        arm2.setPositionTolerance(10);
        arm2.setPositionCoefficient(0.01);
        arm2.setInverted(true);
        waitForStart();
        while(opModeIsActive())
        {
            if(gamepad1.left_bumper)speed = Math.max(speed - 0.0001, 0);
            else if (gamepad1.right_bumper)speed = Math.min(speed + 0.0001, 1.0);
            double diff = gamepad1.right_trigger - gamepad1.left_trigger;
            telemetry.addData("arm1", arm1.motor.getCurrentPosition());
            telemetry.addData("arm2", arm2.motor.getCurrentPosition());
            telemetry.addData("spd", speed*diff);
            arm1.motor.setPower(speed*diff);
            arm2.motor.setPower(speed*diff);
            telemetry.addData("spdafter", speed*diff);
            telemetry.addData("arm1power", arm1.motor.getPower());
            telemetry.addData("arm2power", arm2.motor.getPower());
            telemetry.addData("chicken", "nugget :>");
            telemetry.update();
        }
    }
}
