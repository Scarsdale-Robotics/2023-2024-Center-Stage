package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name="TEST Single Servo Toggle")
public class SingleServoToggleTest extends LinearOpMode {
    public final String SERVO_NAME = "leftClaw";

    @Override
    public void runOpMode() throws InterruptedException {
        Servo servo = hardwareMap.servo.get(SERVO_NAME);
        servo.scaleRange(0, 1);

        waitForStart();

        // CONTROLS:
        // toggle servo        --> triangle
        // inc. bound pos slow --> right trigger
        // dec. bound pos slow --> left trigger
        // inc. bound pos fast --> right trigger + (any) bumper
        // dec. bound pos fast --> right trigger + (any) bumper
        //
        // the bound modified is the "next" bound that will be moved to (ie. if we had just switched
        // to the start bound, now we edit the end bound pos)
        //
        // no bound position modifications may be made while the triangle is pressed

        double toggleChangeFactorSlow = 0.001;
        double toggleChangeFactorFast = 0.01;
        double toggleStart = 0;
        double toggleEnd = toggleChangeFactorSlow;

        boolean buttonDown = false;
        boolean atStart = true;
        while (opModeIsActive()) {
            if (gamepad1.triangle && !buttonDown) {
                servo.setPosition(atStart ? toggleEnd : toggleStart);
                atStart = !atStart;
                buttonDown = true;
            } else if (!gamepad1.triangle) {
                if (atStart)
                    toggleEnd = Math.min(Math.max(toggleEnd + (gamepad1.right_trigger - gamepad1.left_trigger) * (((gamepad1.left_bumper || gamepad1.right_bumper) ? toggleChangeFactorFast : toggleChangeFactorSlow)), toggleStart), 1);
                else
                    toggleStart = Math.min(Math.max(toggleStart + (gamepad1.right_trigger - gamepad1.left_trigger) * (((gamepad1.left_bumper || gamepad1.right_bumper) ? toggleChangeFactorFast : toggleChangeFactorSlow)), toggleChangeFactorSlow), toggleEnd);
                buttonDown = false;
            }
            telemetry.addData("toggle start pos",  toggleStart);
            telemetry.addData("toggle end pos",  toggleEnd);
            telemetry.update();
        }
    }
}
