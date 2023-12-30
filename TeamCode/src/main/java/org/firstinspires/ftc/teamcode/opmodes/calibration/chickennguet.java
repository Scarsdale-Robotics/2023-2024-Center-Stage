package org.firstinspires.ftc.teamcode.opmodes.calibration;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.HardwareRobot;
import org.firstinspires.ftc.teamcode.subsystems.InDepSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RobotSystem;

@TeleOp(name = "chicken nguett")
@Config
public class chickennguet extends LinearOpMode {
    private HardwareRobot hardwareRobot;
    private InDepSubsystem inDep;
    public static double leftPosOpen = 0.6;
    public static double rightPosOpen = 0.25;
    public static double leftPosClosed = 0.21;
    public static double rightPosClosed = 0.6;
    public static double elbowPosRest = 0.17;
    public static double elbowPosFlipped = 0.84;
    public static double wristPos = 0.3;

    private double speed = 1;
    @Override
    public void runOpMode() throws InterruptedException {
        this.hardwareRobot = new HardwareRobot(hardwareMap);
        inDep = new InDepSubsystem(
                hardwareRobot.arm1,
                hardwareRobot.arm2,
                hardwareRobot.elbow,
                hardwareRobot.wrist,
                hardwareRobot.leftClaw,
                hardwareRobot.rightClaw,
                this
        );
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dbTelemetry = dashboard.getTelemetry();
        dbTelemetry.addData("LClaw Pos",inDep.getLeftClawPosition());
        dbTelemetry.addData("RClaw Pos",inDep.getRightClawPosition());
        dbTelemetry.addData("Wrist Pos",inDep.getWristPosition());
        dbTelemetry.addData("Elbow Pos",inDep.getElbowPosition());
        dbTelemetry.update();

        waitForStart();

        boolean clawToggle = false, clawClosed = false;
        boolean elbowToggle = false, elbowClosed = false;

        while(opModeIsActive()) {
            if (gamepad1.triangle && !clawToggle) {
                clawClosed = !clawClosed;
                clawToggle = true;
            }
            if (!gamepad1.triangle) clawToggle = false;

            if (gamepad1.square && !elbowToggle) {
                elbowClosed = !elbowClosed;
                elbowToggle = true;
            }
            if (!gamepad1.square) elbowToggle = false;

            if (clawClosed) {
                inDep.setLeftClawPosition(leftPosClosed);
                inDep.setRightClawPosition(rightPosClosed);
            } else {
                inDep.setLeftClawPosition(leftPosOpen);
                inDep.setRightClawPosition(rightPosOpen);
            }

            if (elbowClosed) {
                inDep.setElbowPosition(elbowPosFlipped);
            } else {
                inDep.setElbowPosition(elbowPosRest);
            }

            inDep.setWristPosition(wristPos);
//            dbTelemetry.addData("LClaw Pos",inDep.getLeftClawPosition());
//            dbTelemetry.addData("RClaw Pos",inDep.getRightClawPosition());
//            dbTelemetry.addData("Wrist Pos",inDep.getWristPosition());
//            dbTelemetry.addData("Elbow Pos",inDep.getElbowPosition());
//            dbTelemetry.update();

            if(gamepad1.left_bumper)speed = Math.max(speed - 0.0001, 0);
            else if (gamepad1.right_bumper)speed = Math.min(speed + 0.0001, 1.0);
            double diff = gamepad1.right_trigger - gamepad1.left_trigger;
            telemetry.addData("armpos", inDep.getArmPosition());
            telemetry.addData("spd", speed*diff);
            inDep.rawPower(speed*diff);
            telemetry.addData("armvelocity", inDep.getArmVelocity());
            telemetry.addData("chicken", "nugget :>");
            telemetry.update();
        }
    }
}
