package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareRobot;
import org.firstinspires.ftc.teamcode.subsystems.CVSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.InDepSubsystem;

@Autonomous(name = "Auto Park Backboard Blue Secondary") //turns second square
public class AutoParkBackboardBlueSecondary extends LinearOpMode {
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
                robot.cameraName,drive, telemetry, false);
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
        Thread.sleep(15000); //wait 15 sec for teammate to do auto

        drive.driveByEncoder(0, 0.5, 0, 150); // moving forward toward the pixel placing area
        drive.driveByEncoder(0, 0, -1, 300);  // turn left
        drive.driveByEncoder(0, 0.5, 0, 200); // continue moving forward toward the parking area
        stop();
    }

    public void stop(DriveSubsystem drive) {
        drive.driveByEncoder(0, 0, 0, 0);
    }
}
