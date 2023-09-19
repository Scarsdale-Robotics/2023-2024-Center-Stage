package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareRobot;
import org.firstinspires.ftc.teamcode.subsystems.core.DriveSubsystem;

// Set the name for the OpMode that will appear on the driver station
@Autonomous(name = "Example Auto OpMode")
// Create class and basically include methods for opmode
public class ExAutoOp extends LinearOpMode {
    @Override
    // The "Main" code will go in here
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

        waitForStart();


    }

    public void AutoLeftTop(){
        HardwareRobot robot = new HardwareRobot(hardwareMap);
        DriveSubsystem drive = new DriveSubsystem(
                robot.leftFront,
                robot.rightFront,
                robot.leftBack,
                robot.rightBack,
                robot.imu,
                this
        );
        //pretend april tag location has been found, 0 = left, 1 = center, 2 = right
        drive.driveByEncoder(0, 1, 0, 5);
        //place pixel on spike (we don't have claw yet)
        drive.driveByEncoder(0,0,-90,0); // Turning left
        drive.driveByEncoder(0, 1, 0, 10); // move towards parking spot
        //park
    }

    public void AutoRightTop(){
        HardwareRobot robot = new HardwareRobot(hardwareMap);
        DriveSubsystem drive = new DriveSubsystem(
                robot.leftFront,
                robot.rightFront,
                robot.leftBack,
                robot.rightBack,
                robot.imu,
                this
        );
        //pretend april tag location has been found, 0 = left, 1 = center, 2 = right
        drive.driveByEncoder(0, 1, 0, 5);
        //place pixel on spike (we don't have claw yet)
        drive.driveByEncoder(0,0,90,0); // Turning right
        drive.driveByEncoder(0, 1, 0, 10); // move towards parking spot
        //park
    }
}