package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareRobot;
import org.firstinspires.ftc.teamcode.subsystems.CVSubsystem;
import org.firstinspires.ftc.teamcode.SpeedCoefficients;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.InDepSubsystem;

@Autonomous(name = "Auto Backboard Red")
public class AutoBackboardRed extends LinearOpMode {
    @Override
    public void runOpMode() {
        HardwareRobot robot = new HardwareRobot(hardwareMap);
        DriveSubsystem drive = new DriveSubsystem(
                robot.leftFront,
                robot.rightFront,
                robot.leftBack,
                robot.rightBack,
                robot.imu,
                this
        );
        CVSubsystem cvSubsystem = new CVSubsystem(robot.camera, drive);
        SpeedCoefficients speedCoefficients = new SpeedCoefficients();

        // Initialize InDepSubsystem with the hardware components from HardwareRobot
        InDepSubsystem inDep = new InDepSubsystem(
                robot.arm,
                robot.claw,
                robot.wrist,
                this,
                telemetry
        );

        waitForStart();

        int propLocation = cvSubsystem.getTeamPropLocation(true); // Changed to true for red team

        if (propLocation == 0) { // left
            drive.driveByEncoder(0, 0.5, 0, 550);
            drive.driveByEncoder(0, 0, 1, 300); // Changed to turn right
            drive.driveByEncoder(0, 0.5, 0, 250);
            inDep.open(); // open claw
            drive.driveByEncoder(0, 0, 0, -400);
            drive.driveByEncoder(0, 0, -1, 300); // Changed to turn left
            drive.driveByEncoder(0, 0.5, 0, 250);
            stop();
        }
        else if (propLocation == 1) { // center
            drive.driveByEncoder(0, 0.7, 0, 1100);
            drive.driveByEncoder(0, 0, -90, 0); // Changed to turn left
            drive.driveByEncoder(0, 0.7, 0, 100);
            drive.driveByEncoder(0, 0, 90, 0); // Changed to turn right
            drive.driveByEncoder(0, 0.7, 0, 150);
            inDep.open(); // open claw
            drive.driveByEncoder(0, 0.7, 0, -250);
            drive.driveByEncoder(0, 0, 90, 0); // Changed to turn right
            cvSubsystem.moveToAprilTag(0);
        }
        else if (propLocation == 2) { // right
            drive.driveByEncoder(0, 0.7, 0, 550);
            drive.driveByEncoder(0, 0, -90, 0); // Changed to turn left
            drive.driveByEncoder(0, 0.7, 0, 100);
            inDep.open(); // open claw
            drive.driveByEncoder(0, 0.7, 0, -250);
            drive.driveByEncoder(0, 0, 180, 0); // 180 turns are the same for both sides
            cvSubsystem.moveToAprilTag(0);
        }
    }

    public void stop(DriveSubsystem drive) {
        drive.driveByEncoder(0, 0, 0, 0);
    }
}