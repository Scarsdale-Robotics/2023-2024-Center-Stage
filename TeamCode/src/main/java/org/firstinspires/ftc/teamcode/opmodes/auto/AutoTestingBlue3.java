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
        CVSubsystem cv = new CVSubsystem(robot.camera,
                robot.cameraName,drive, telemetry, false, this);

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
        int propLocation = cv.getTeamPropLocation(); // 0 = left, 1 = center, 2 = right
        //only run this when 2 works and vals are updated. may hit truss
        if (propLocation == 0) {
            // left
            drive.driveByEncoder(0, -0.3, 0, 1200); // moving forward toward the pixel placing area
            drive.driveByEncoder(0, 0.3, 0, 1); // brake
            drive.driveByEncoder(0, 0, 0.5, 885);  // turn left
            drive.driveByEncoder(0, -0.3, 0, 100); // moving forward to the spike mark tape
            drive.driveByEncoder(0, 0.3, 0, 1); // brake
            inDep.open(); // open claw to place the pixel
            inDep.changeElevation(1500); // raise claw
            drive.driveByEncoder(0, -0.3, 0, 3550); // moving forward to the spike mark tape
            inDep.changeElevation(-1500); // lower claw
        } else if (propLocation == 1) {
            // center
            drive.driveByEncoder(0, -0.3, 0, 1300); // moving forward toward the pixel placing area
            drive.driveByEncoder(0, 0.3, 0, 1); // brake
            inDep.open(); // open claw to place the pixel
            drive.driveByEncoder(0, 0.3, 0, 100); // move backwards to the spike mark tape
            drive.driveByEncoder(0, -0.3, 0, 1); // brake
            drive.driveByEncoder(0, 0, 0.5, 885);  // turn left
            drive.driveByEncoder(0, -0.3, 0, 3550); // moving forward to the spike mark tape
            inDep.changeElevation(-1500); // lower claw
        } else {
            // right
            drive.driveByEncoder(0, -0.3, 0, 1200); // moving forward toward the pixel placing area
            drive.driveByEncoder(0, 0.5, 0, 1); // brake
            drive.driveByEncoder(0, 0, -0.5, 885);  // turn right
            drive.driveByEncoder(0, -0.3, 0, 100); // moving forward to the spike mark tape
            drive.driveByEncoder(0, 0.3, 0, 1); // brake
            inDep.open(); // open claw to place the pixel
            drive.driveByEncoder(0, 0.3, 0, 100); // moving back to center
            drive.driveByEncoder(0, -0.3, 0, 1); // brake
            //drive.driveByEncoder(0, 0, 0.5, 885);  // turn left 180º (only needed to place pixel)
            drive.driveByEncoder(0, 0.3, 0, 3550); // moving backward to the spike mark tape
            inDep.changeElevation(-1500); // lower claw
        }
        stop();



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
