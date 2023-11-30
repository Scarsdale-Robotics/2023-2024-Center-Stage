package org.firstinspires.ftc.teamcode.opmodes.auto.OtherPark;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareRobot;
import org.firstinspires.ftc.teamcode.opmodes.auto.AutoUtility;
import org.firstinspires.ftc.teamcode.subsystems.CVSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.InDepSubsystem;

@Autonomous(name = "Different Park Auto Far Blue")
public class AutoDifferentParkFarBlue extends LinearOpMode {
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

        AutoUtility autoUtil = new AutoUtility(cv, drive, inDep, this);

        inDep.close();

        waitForStart();

        Thread.sleep(500);

        //Start actual Auto now | choose cv or manual prop location
        int propLocation = autoUtil.placePurplePixelBlue(500, false,telemetry);
        //propLocation = -1;

        //only run this when 2 works and vals are updated. may hit truss
        if (propLocation == 0) {

            inDep.changeElevation(2200); // raise claw

            drive.driveByEncoder(0, 0.3, 0, 500); // move backwards
            drive.driveByEncoder(0.3, 0, 0, 1500); // strafe right to place pixel correctly
            inDep.changeElevation(-2200); // lower claw
            //drive.driveByEncoder(0, -0.3, 0, 500); // move forwards

            drive.driveByEncoder(0, -0.3, 0, 3185); // moving forward to the spike mark tape4
            drive.driveByEncoder(0, 0, 0.5, 1830);  // turn left 180º (only needed to place pixel)
            drive.driveByEncoder(0.3, 0, 0, 200); // strafe left to finish parking
            drive.driveByEncoder(0, 0.3, 0, 400); // moving backward to the spike mark tape
            drive.driveByEncoder(0, 0.3, 0, 75+75+75); // moving backward to the spike mark tape


        } else if (propLocation == 1) {
            // center
            inDep.changeElevation(2200); // raise claw
            drive.driveByEncoder(0, 0.3, 0, 950);
            //drive.driveByEncoder(0, -0.3, 0, 1300); // move backwards to the spike mark tape
//            drive.driveByEncoder(0, -0.3, 0, 1); // brake
            drive.driveByEncoder(0, 0, -0.5, 855);  // turn right
            inDep.changeElevation(-2200); // lower claw
            //drive.driveByEncoder(-0.3, 0, 0, 300); // strafe right to the spike mark tape
            drive.driveByEncoder(0, 0.3, 0, 3750); // moving forward to the spike mark tape
            //drive.driveByEncoder(0, 0, 0.5, 1800);  // turn left 180º (only needed to place pixel)
            //drive.driveByEncoder(0, 0.3, 0, 525); // moving backward to the spike mark tape
        } else if (propLocation == 2) {
            // right
            inDep.changeElevation(2000); // raise claw
            drive.driveByEncoder(-0.3, 0, 0, 1550); // strafe right
            drive.driveByEncoder(0, 0.3, 0, 3750); // moving backward to the spike mark tape
            inDep.changeElevation(-2000); // lower claw
            //inDep.changeElevation(-1500); // lower claw
        }
        stop();
    }

    public void stop(DriveSubsystem drive) {
        drive.driveByEncoder(0, 0, 0, 0);
    }
}
