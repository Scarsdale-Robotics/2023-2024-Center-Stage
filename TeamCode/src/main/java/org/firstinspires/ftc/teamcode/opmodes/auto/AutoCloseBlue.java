package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.teamcode.HardwareRobot;
import org.firstinspires.ftc.teamcode.subsystems.CVSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.InDepSubsystem;

@Autonomous(name = "Auto Close Blue")
public class AutoCloseBlue extends LinearOpMode {
    private DriveSubsystem drive;
    private InDepSubsystem inDep;
    @Override
    // The "Main" code will go in here
    public void runOpMode() throws InterruptedException {
        HardwareRobot robot = new HardwareRobot(hardwareMap);
        drive = new DriveSubsystem(
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
        inDep = new InDepSubsystem(
                robot.arm,
                robot.claw,
                robot.wrist,
                this,
                telemetry
        );

        AutoUtility autoUtil = new AutoUtility(cv, drive, inDep, this);


        waitForStart();

        //Start actual Auto now // pretend april tag location has been found, 0 = left, 1 = center, 2 = right
//        int propLocation = cvSubsystem.getTeamPropLocation(); // 0 = left, 1 = center, 2 = right

        drive.driveByEncoder(0, -0.3, 0, 111); // move forward
        drive.driveByEncoder(0, 0, 0.5, 56); // turn left to counter initial offset kinda but not fully, intentionally
        drive.driveByEncoder(0, -0.3, 0, 333); // move forward
        drive.driveByEncoder(-0.3, 0, 0, 25);
        int propLocation = autoUtil.placePurplePixelBlue(-250, true);


        if (propLocation == 0) { // left
            inDep.changeElevation(2000);
            drive.driveByEncoder(0.3, 0, 0, 1100); // move left

            // nathan bb function
            drive.driveByEncoder(0, -0.3, 0, 1111); // move forward a bit
            drive.driveByEncoder(0, 0, -0.5, 1800); // perform 180
            drive.driveByEncoder(0, 0.3, 0, 888); // move backwards to park
            inDep.close();

            inDep.changeElevation(-2000);
            stop();
            // autoUtil.moveToAprilTag(0); //temporary because we don't have april tag id
            // park
        }

        else if (propLocation == 1) { // center
            //inDep.changeElevation(10); // raise claw
            inDep.changeElevation(2000);
            drive.driveByEncoder(0, 0, 0.5, 850); // turn left 90 degs
            drive.driveByEncoder(0.3, 0, 0, 1000); // move left

            // nathan bb function
            drive.driveByEncoder(0, -0.3, 0, 1111); // move forward a bit
            drive.driveByEncoder(0, 0, -0.5, 1800); // perform 180
//        drive.driveByEncoder(0.3, 0, 0, 2222); // move left
            drive.driveByEncoder(0, 0.3, 0, 888); // move backwards to park
            inDep.close();

            inDep.changeElevation(-2000);
//            //inDep.changeElevation(-10); // lower claw
//            inDep.open(); // open claw to place the pixel
//           1 //inDep.changeElevation(10); // raise claw
//            drive.driveByEncoder(0, 0.5, 0, -250); // moving back to pixel placing area
//            drive.driveByEncoder(0, 0, -1, 300);  // turn left
//            drive.driveByEncoder(0, 0.5, 0, 1500); // continue moving forward toward the parking area
            stop();
            // autoUtil.moveToAprilTag(1); //temporary because we don't have april tag id for center
            // park
        }

        else if (propLocation == 2) { // right
            //inDep.changeElevation(10); // raise claw
            inDep.changeElevation(2000);
            drive.driveByEncoder(0, 0.3, 0, 222); // move backwards a bit
            drive.driveByEncoder(-0.3, 0, 0, 1222); // move right

            // nathan bb function
            drive.driveByEncoder(0, 0.3, 0, 1500); // move backwards to park
            inDep.close();

            inDep.changeElevation(-2000);
//            drive.driveByEncoder(0, 0.5, 0, 550); // moving forward more than the center path
//            drive.driveByEncoder(0, 0, 1, 300);  // turn right
//            drive.driveByEncoder(0, 0.5, 0, 250); // moving forward to the spike mark tape
//            //inDep.changeElevation(-10); // lower claw
//            inDep.open(); // open claw to place the pixel
//            //inDep.changeElevation(10);  // raise claw
//            drive.driveByEncoder(0, 0, -1, 600);  // turn left
//            drive.driveByEncoder(0, 0.5, 0, 1750); // continue moving forward toward the parking area
            stop();
            // autoUtil.moveToAprilTag(2); //temporary because we don't have april tag id for right
            // park
        } else {

        }

    }

    public void stop(DriveSubsystem drive) {
        drive.driveByEncoder(0, 0, 0, 0);
    }
}
