package org.firstinspires.ftc.teamcode.opmodes.auto.backup;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareRobot;
import org.firstinspires.ftc.teamcode.subsystems.CVSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.InDepSubsystem;

@Autonomous(name = "Auto Backup Far Blue") //turns first square
public class AutoBackupFarBlue extends LinearOpMode {
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

        int moveOffset = 500;
        boolean isCloseBlue = false;

        //Technically CV
        drive.driveByEncoder(0, -0.3, 0, 720 + moveOffset); // moving forward toward the pixel placing area
        drive.driveByEncoder(0, 0.3, 0, 1); // brake
        drive.driveByEncoder(0, 0, 0.5, 888 + (isCloseBlue ? -50 : 18));  // turn left 90 degrees
        drive.driveByEncoder(0, -0.3 * (isCloseBlue ? -1 : 1), 0, 50 + (isCloseBlue ? 50 : 0)); // moving forward (or backward) to the spike mark tape
        drive.driveByEncoder(0, 0.3, 0, 1); // brake
        drive.driveByEncoder(-0.3, 0, 0, 300 + (isCloseBlue ? -50 : 0)); // strafe right to place pixel correctly

        Thread.sleep(1000); //wait 1 sec for teammate to do auto

        //Rest of Auto
        inDep.changeElevation(2200); // raise claw
        drive.driveByEncoder(0, 0.3, 0, 500); // move backwards
        drive.driveByEncoder(-0.3, 0, 0, 1100); // strafe right to place pixel correctly
        inDep.changeElevation(-2200); // lower claw
        drive.driveByEncoder(0, -0.3, 0, 500); // move forwards
        drive.driveByEncoder(0, -0.3, 0, 3185); // moving forward to the spike mark tape4
        drive.driveByEncoder(0, 0, 0.5, 1830);  // turn left 180º (only needed to place pixel)
        //drive.driveByEncoder(0.3, 0, 0, 200); // strafe left to finish parking
        drive.driveByEncoder(0, 0.3, 0, 400); // moving backward to the spike mark tape
        drive.driveByEncoder(0, 0.3, 0, 75+75+75); // moving backward to the spike mark tape
        stop();
    }

    public void stop(DriveSubsystem drive) {
        drive.driveByEncoder(0, 0, 0, 0);
    }
}
