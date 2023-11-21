package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareRobot;
import org.firstinspires.ftc.teamcode.subsystems.CVSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.InDepSubsystem;

@Autonomous(name = "Auto Testing Blue 3")
public class AutoTestingBlue3 extends LinearOpMode {
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
        //SpeedCoefficients.setMode(SpeedCoefficients.MoveMode.MODE_FAST);


        waitForStart();

        //Start actual Auto now // pretend april tag location has been found, 0 = left, 1 = center, 2 = right
        //int propLocation = cvSubsystem.getTeamPropLocation(false); // 0 = left, 1 = center, 2 = right
        //only run this when 2 works and vals are updated. may hit truss
        drive.driveByEncoder(0, -0.3, 0, 1200); // moving forward toward the pixel placing area
        drive.driveByEncoder(0, 0, 0.5, 875);  // turn left
        //drive.driveByEncoder(0, 0.5, 0, 1); // move backwards to brake for tesin
        inDep.changeElevation(150); // raise claw - make 100 if doesnt work
        drive.driveByEncoder(0, -0.3, 0, 100); // moving forward to the spike mark tape
        drive.driveByEncoder(0, 0.3, 0, 1); // moving forward to the spike mark tape
        inDep.open(); // open claw to place the pixel
        drive.driveByEncoder(0, -0.3, 0, 3000); // moving forward to the spike mark tape
        inDep.changeElevation(-150); // lower claw

        //inDep.changeElevation(10); // raise



//        if (propLocation == 0) { // left
//            inDep.changeElevation(10); // raise claw
//            drive.driveByEncoder(0, 0.5, 0, 550); // moving forward toward the pixel placing area
//            drive.driveByEncoder(0, 0, -1, 300);  // turn left
//            drive.driveByEncoder(0, 0.5, 0, 250); // moving forward to the spike mark tape
//            inDep.changeElevation(-10); // lower claw
//            inDep.open(); // open claw to place the pixel
//            inDep.changeElevation(10); // raise claw
//            drive.driveByEncoder(0, 0.5, 0, 250); // continue moving forward toward the parking area
//            stop();
//            // autoUtil.moveToAprilTag(0); //temporary because we don't have april tag id
//            // park
//        }
//
//        else if (propLocation == 1) { // center
//            inDep.changeElevation(10); // raise claw
//            drive.driveByEncoder(0, 0.5, 0, 850); // moving forward to spike mark tape
//            inDep.changeElevation(-10); // lower claw
//            inDep.open(); // open claw to place the pixel
//            inDep.changeElevation(10); // raise claw
//            drive.driveByEncoder(0, 0.5, 0, -250); // moving back to pixel placing area
//            drive.driveByEncoder(0, 0, -1, 300);  // turn left
//            drive.driveByEncoder(0, 0.5, 0, 500); // continue moving forward toward the parking area
//            stop();
//            // autoUtil.moveToAprilTag(1); //temporary because we don't have april tag id for center
//            // park
//        }
//
//        else if (propLocation == 2) { // right
//            inDep.changeElevation(10); // raise claw
//            drive.driveByEncoder(0, 0.5, 0, 550); // moving forward more than the center path
//            drive.driveByEncoder(0, 0, 1, 300);  // turn right
//            drive.driveByEncoder(0, 0.5, 0, 250); // moving forward to the spike mark tape
//            inDep.changeElevation(-10); // lower claw
//            inDep.open(); // open claw to place the pixel
//            inDep.changeElevation(10);  // raise claw
//            drive.driveByEncoder(0, 0, -1, 600);  // turn left
//            drive.driveByEncoder(0, 0.5, 0, 750); // continue moving forward toward the parking area
//            stop();
//            // autoUtil.moveToAprilTag(2); //temporary because we don't have april tag id for right
//            // park
//        }

    }

    public void stop(DriveSubsystem drive) {
        drive.driveByEncoder(0, 0, 0, 0);
    }
}
