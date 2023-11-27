package org.firstinspires.ftc.teamcode.opmodes.auto.backup;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareRobot;
import org.firstinspires.ftc.teamcode.subsystems.CVSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.InDepSubsystem;

@Autonomous(name = "Auto Backup Close Red") //turns first square
public class AutoBackupCloseRed extends LinearOpMode {
    @Override
    // The "Main" code will go in here
    public void runOpMode() throws InterruptedException {
        HardwareRobot robot = new HardwareRobot(hardwareMap);
        DriveSubsystem drive = new DriveSubsystem(
                robot.leftFront,
                robot.rightFront,
                robot.leftBack,
                robot.rightBack,
                robot.imu,
                this
        );
        CVSubsystem cvSubsystem = new CVSubsystem(robot.camera,
                robot.cameraName,drive, telemetry, false, this);
        // Assuming this is in your main OpMode class
        HardwareRobot hardwareRobot = new HardwareRobot(hardwareMap);

        // Initialize InDepSubsystem with the hardware components from HardwareRobot
        InDepSubsystem inDep = new InDepSubsystem(
                robot.arm,
                robot.claw,
                robot.wrist,
                this,
                telemetry
        );
        waitForStart();

        //Theoretically is CV
        drive.driveByEncoder(0, -0.3, 0, 800 + 600); // moving forward toward the pixel placing area
        drive.driveByEncoder(0, 0.5, 0, 1); // brake
        drive.driveByEncoder(0, 0, 0.5, 915 -27);  // turn left
        drive.driveByEncoder(0, -0.3, 0, 85); // moving back/approach
        drive.driveByEncoder(0, -0.3, 0, 10); // brake
        drive.driveByEncoder(-0.3, 0, 0, 0); // strafe right

        Thread.sleep(1000); //wait 1 sec for teammate to do auto

        //Rest of Auto
        inDep.changeElevation(2000);
        drive.driveByEncoder(0, 0.3, 0, 250); // move backwards a bit
        drive.driveByEncoder(0.3, 0, 0, 1400); // move left
        drive.driveByEncoder(0, 0.3, 0, 1200); // move backwards to park
        inDep.close();

        inDep.changeElevation(-2000);
        stop();
    }

    public void stop(DriveSubsystem drive) {
        drive.driveByEncoder(0, 0, 0, 0);
    }
}
