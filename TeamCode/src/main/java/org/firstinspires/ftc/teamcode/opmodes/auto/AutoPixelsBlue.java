package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareRobot;
import org.firstinspires.ftc.teamcode.subsystems.CVSubsystem;
import org.firstinspires.ftc.teamcode.SpeedCoefficients;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.InDepSubsystem;

@Autonomous(name = "Auto Backboard Blue")
public class AutoPixelsBlue extends LinearOpMode {
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
        CVSubsystem cvSubsystem = new CVSubsystem(robot.camera, drive);
        SpeedCoefficients speedCoefficients = new SpeedCoefficients();
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

        //Start actual Auto now // pretend april tag location has been found, 0 = left, 1 = center, 2 = right
        int propLocation = cvSubsystem.getTeamPropLocation(false); // 0 = left, 1 = center, 2 = right


        if (propLocation == 0) { // left
            drive.driveByEncoder(0, 0.5, 0, 550); // moving
            drive.driveByEncoder(0, 0, -1,300); // left
            drive.driveByEncoder(0, 0.5, 0, 250); // moving
            inDep.open(); // open claw
            drive.driveByEncoder(0,0,0,-400); // moving backwards
            drive.driveByEncoder(0,0,1,300); // turn right
            drive.driveByEncoder(0,0.5,0,250); // moving
            stop();
//                autoUtil.moveToAprilTag(0); //temporary bcs don't have april tag id
            //park
        }
        else if (propLocation == 1) { // center
            drive.driveByEncoder(0, 0.7, 0, 1100); // moving
            drive.driveByEncoder(0, 0, 90,0 ); // turn right
            drive.driveByEncoder(0, 0.7, 0, 100); // moving
            drive.driveByEncoder(0, 0, -90, 0); // turn left
            drive.driveByEncoder(0,0.7,0, 150); // moving
            inDep.open(); //open claw
            drive.driveByEncoder(0,0.7,0,-250); // moving backwards
            drive.driveByEncoder(0,0,-90,0); // turn left
            cvSubsystem.moveToAprilTag(0); //temporary bcs don't have april tag id
            //park
        }
        else if (propLocation == 2) { // right
            drive.driveByEncoder(0, 0.7, 0, 550); // moving
            drive.driveByEncoder(0, 0, 90,0 ); // turn right
            drive.driveByEncoder(0, 0.7, 0, 100); // moving
            inDep.open(); //open claw
            drive.driveByEncoder(0, 0.7, 0, -250); // moving backwards
            drive.driveByEncoder(0, 0, 180, 0); // turning
            cvSubsystem.moveToAprilTag(0); //temporary bcs don't have april tag id
            //park
        }
    }

    public void stop(DriveSubsystem drive) {
        drive.driveByEncoder(0, 0, 0, 0);
    }
}
