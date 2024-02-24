package org.firstinspires.ftc.teamcode.opmodes.calibration;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareRobot;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.EndgameSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.InDepSubsystem;

@TeleOp(name = "Subsystem Tester")
public class SubsystemTester extends LinearOpMode {
    private HardwareRobot hardwareRobot;
    private DriveSubsystem drive;
    private InDepSubsystem inDep;
    private EndgameSubsystem endgame;

    @Override
    public void runOpMode() throws InterruptedException {
        this.hardwareRobot = new HardwareRobot(hardwareMap);
        inDep = new InDepSubsystem(
                hardwareRobot.leftArm,
                hardwareRobot.rightArm,
                hardwareRobot.elbow,
                hardwareRobot.wrist,
                hardwareRobot.leftClaw,
                hardwareRobot.rightClaw,
                this
        );
        drive = new DriveSubsystem(
                hardwareRobot.leftFront,
                hardwareRobot.rightFront,
                hardwareRobot.leftBack,
                hardwareRobot.rightBack,
                hardwareRobot.imu,
                this
        );
        endgame = new EndgameSubsystem(hardwareRobot.drone);

        waitForStart();

        telemetry.clearAll();
        while (opModeIsActive() && !gamepad1.right_bumper) {
            drive.driveWithMotorPowers(gamepad1.left_stick_x,0,0,0);
            telemetry.addData("leftFront:",gamepad1.left_stick_x);
            telemetry.update();
        } while (opModeIsActive() && gamepad1.right_bumper);

        telemetry.clearAll();
        while (opModeIsActive() && !gamepad1.right_bumper) {
            drive.driveWithMotorPowers(0,gamepad1.left_stick_x,0,0);
            telemetry.addData("rightFront:",gamepad1.left_stick_x);
            telemetry.update();
        } while (opModeIsActive() && gamepad1.right_bumper);

        telemetry.clearAll();
        while (opModeIsActive() && !gamepad1.right_bumper) {
            drive.driveWithMotorPowers(0,0,gamepad1.left_stick_x,0);
            telemetry.addData("leftBack:",gamepad1.left_stick_x);
            telemetry.addData("leftBack Pos:",drive.getLBPosition());
            telemetry.update();
        } while (opModeIsActive() && gamepad1.right_bumper);

        telemetry.clearAll();
        while (opModeIsActive() && !gamepad1.right_bumper) {
            drive.driveWithMotorPowers(0,0,0,gamepad1.left_stick_x);
            telemetry.addData("rightBack:",gamepad1.left_stick_x);
            telemetry.addData("rightBack Pos:",drive.getRBPosition());
            telemetry.update();
        } while (opModeIsActive() && gamepad1.right_bumper);

        telemetry.clearAll();
        while (opModeIsActive() && !gamepad1.right_bumper) {
            inDep.rawPower(gamepad1.left_stick_x);
            telemetry.addData("rawPower:",gamepad1.left_stick_x);
            telemetry.addData("left arm Pos:",inDep.getLeftArmPosition());
            telemetry.addData("right arm Pos:",inDep.getRightArmPosition());
            telemetry.update();
        } while (opModeIsActive() && gamepad1.right_bumper);

        telemetry.clearAll();
        while (opModeIsActive() && !gamepad1.right_bumper) {
            inDep.setElbowPosition(gamepad1.left_stick_x/5+0.5);
            inDep.setWristPosition(gamepad1.left_stick_x/5+0.5);
            inDep.setLeftClawPosition(gamepad1.left_stick_x/5+0.5);
            inDep.setRightClawPosition(gamepad1.left_stick_x/5+0.5);
            telemetry.addData("elbow servoPos:",inDep.getElbowPosition());
            telemetry.addData("wrist servoPos:",inDep.getWristPosition());
            telemetry.addData("left claw servoPos:",inDep.getLeftClawPosition());
            telemetry.addData("right claw servoPos:",inDep.getRightClawPosition());
            telemetry.update();
        } while (opModeIsActive() && gamepad1.right_bumper);

        telemetry.clearAll();
        while (opModeIsActive() && !gamepad1.right_bumper) {
            endgame.setPower(gamepad1.left_stick_x/5+0.5);
            telemetry.addData("endgame servoPos:",gamepad1.left_stick_x/5+0.5);
            telemetry.update();
        } while (opModeIsActive() && gamepad1.right_bumper);

    }
}
