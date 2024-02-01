package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.TeleOpUtil;

@TeleOp(name = "RED Drive TeleOp")
public class DriveTeleOpRed extends LinearOpMode {
    @Override
    public void runOpMode() {
        TeleOpUtil teleOp = new TeleOpUtil(hardwareMap, telemetry, true, gamepad1, gamepad2, this);
        waitForStart();
        while (opModeIsActive()) {
            teleOp.tick();
//            telemetry.addData("Arm pos: ", teleOp.robot.arm.motor.getCurrentPosition());
//            telemetry.addData("Wrist pos: ", teleOp.robot.wrist.getPosition());
//            telemetry.addData("Claw pos: ", teleOp.robot.claw.getPosition());
//            telemetry.addData("arm power: ", teleOp.robot.arm.motor.getPower());
//            telemetry.addData("Speed is Fast: ", teleOp.speedIsFast);
//            telemetry.addData("Omni Mode:", teleOp.omniMode);
//            telemetry.update();
        }
    }
}
