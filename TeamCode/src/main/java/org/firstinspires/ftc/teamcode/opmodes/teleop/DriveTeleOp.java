package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.checkerframework.checker.units.qual.Speed;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.HardwareRobot;
import org.firstinspires.ftc.teamcode.SpeedCoefficients;
import org.firstinspires.ftc.teamcode.subsystems.InDepSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.CVSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TeleOpUtil;

@TeleOp(name = "Drive TeleOp")
public class DriveTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() {
        TeleOpUtil teleOp = new TeleOpUtil(hardwareMap, telemetry, false, gamepad1, gamepad2, this);
        waitForStart();
        while (opModeIsActive()) {
            teleOp.tick();
            telemetry.addData("Arm pos: ", teleOp.robot.arm.motor.getCurrentPosition());
            telemetry.addData("Wrist pos: ", teleOp.robot.wrist.getPosition());
            telemetry.addData("Claw pos: ", teleOp.robot.claw.getPosition());
            telemetry.addData("arm power: ", teleOp.robot.arm.motor.getPower());
            telemetry.addData("Speed is Fast: ", teleOp.speedIsFast);
            telemetry.addData("Omni Mode:", teleOp.omniMode);
            telemetry.update();
        }
    }
}
