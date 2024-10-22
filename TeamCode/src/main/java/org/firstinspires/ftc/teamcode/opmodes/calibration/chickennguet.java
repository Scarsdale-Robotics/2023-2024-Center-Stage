package org.firstinspires.ftc.teamcode.opmodes.calibration;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.HardwareRobot;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.EndgameSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.InDepSubsystem;

@TeleOp(name = "chicken nguett")
@Config
public class chickennguet extends LinearOpMode {
    private HardwareRobot hardwareRobot;
    private DriveSubsystem drive;
    private InDepSubsystem inDep;
    private EndgameSubsystem endgame;
    public static double leftPosOpen = 0.6;
    public static double rightPosOpen = 0.25;
    public static double leftPosClosed = 0.21;
    public static double rightPosClosed = 0.6;
    public static double elbowPosRest = 0.10;
    public static double elbowPosFlipped = 0.74;
    public static double wristPos = 0.25;
    public static double dronePos = 0.0;

    private double speed = 1;
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
                this,
                false
        );
        endgame = new EndgameSubsystem(hardwareRobot.drone);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dbTelemetry = dashboard.getTelemetry();
        dbTelemetry.addData("LClaw Pos",inDep.getLeftClawPosition());
        dbTelemetry.addData("RClaw Pos",inDep.getRightClawPosition());
        dbTelemetry.addData("Wrist Pos",inDep.getWristPosition());
        dbTelemetry.addData("Elbow Pos",inDep.getElbowPosition());
        dbTelemetry.addData("LArm Pos", hardwareRobot.leftArm.getCurrentPosition());
        dbTelemetry.addData("RArm Pos", hardwareRobot.rightArm.getCurrentPosition());
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
            endgame.setPower(dronePos);
            dbTelemetry.addData("LClaw Pos",inDep.getLeftClawPosition());
            dbTelemetry.addData("RClaw Pos",inDep.getRightClawPosition());
            dbTelemetry.addData("Wrist Pos",inDep.getWristPosition());
            dbTelemetry.addData("Elbow Pos",inDep.getElbowPosition());
            dbTelemetry.addData("LArm Pos", hardwareRobot.leftArm.getCurrentPosition());
            dbTelemetry.addData("RArm Pos", hardwareRobot.rightArm.getCurrentPosition());
            telemetry.addData("LArm Pos", hardwareRobot.leftArm.getCurrentPosition());
            telemetry.addData("RArm Pos", hardwareRobot.rightArm.getCurrentPosition());
            dbTelemetry.addData("Drone Pos", endgame.getPower());
            dbTelemetry.update();
            telemetry.update();

            if(gamepad1.left_bumper)speed = Math.max(speed - 0.0001, 0);
            else if (gamepad1.right_bumper)speed = Math.min(speed + 0.0001, 1.0);
            double diff = gamepad1.right_trigger - gamepad1.left_trigger;
            telemetry.addData("armpos", inDep.getLeftArmPosition());
            telemetry.addData("spd", speed*diff);
            hardwareRobot.leftArm.motor.setPower(0.25 * speed*diff);
            hardwareRobot.rightArm.motor.setPower(0.25 * speed*diff);
            telemetry.addData("armvelocity", inDep.getLeftArmVelocity());
            telemetry.addData("chicken", "nugget :>");
            telemetry.update();


//            double x_stick = gamepad1.left_stick_x,
//                    y_stick = gamepad1.left_stick_y,
//                    x_turn = gamepad1.right_stick_x,
//                    K = SpeedCoefficients.getAutonomousDriveSpeed();
//
//            if (Math.abs(x_stick)>0.01 || Math.abs(y_stick)>0.01) {
////                double theta = Math.atan2(y_stick,x_stick);
////                double L = Math.sin(theta - Math.PI / 4),
////                        R = Math.sin(theta + Math.PI / 4),
////                        K = SpeedCoefficients.getAutonomousDriveSpeed();
////                drive.driveWithMotorPowers(R * K, L * K, L * K, R * K);
//            } else {
//                x_stick = 0;
//                y_stick = 0;
//            }
//            drive.driveFieldCentric(-x_stick * K, y_stick * K, -x_turn * K);

        }
    }
}
