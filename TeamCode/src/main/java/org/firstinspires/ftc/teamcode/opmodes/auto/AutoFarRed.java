package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareRobot;
import org.firstinspires.ftc.teamcode.subsystems.CVSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.InDepSubsystem;

@Autonomous(name = "Auto Far Red")
public class AutoFarRed extends LinearOpMode {
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
                robot.cameraName,drive, telemetry, true, this);

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
        //Thread.sleep(0);

        drive.driveByEncoder(0, -0.3, 0, 30); // move forward
        //drive.driveByEncoder(0, 0, 0.5, 56); // turn left to counter initial offset kinda but not fully, intentionally
        drive.driveByEncoder(-0.3, 0, 0, 400-25); // strafe right
        //Start actual Auto now | choose cv or manual prop location
        Thread.sleep(1000);
        int propLocation = autoUtil.placePurplePixelRed(0, false,telemetry);
        //propLocation = -1; // removes parking

        //only run this when 2 works and vals are updated. may hit truss
        if (propLocation == 2) {
            // right
            /*
            drive.driveByEncoder(0, -0.3, 0, 1200); // moving forward toward the pixel placing area
            drive.driveByEncoder(0, 0.3, 0, 1); // brake
            drive.driveByEncoder(0, 0, 0.5, 880);  // turn left
            drive.driveByEncoder(0, -0.3, 0, 50); // moving forward to the spike mark tape
            drive.driveByEncoder(0, 0.3, 0, 1); // brake
            drive.driveByEncoder(-0.3, 0, 0, 50); // strafe right to place pixel correctly
            inDep.open(); // open claw to place the pixel
            inDep.changeElevation(2500); // raise claw
            drive.driveByEncoder(0, -0.3, 0, 2785); // moving forward to the spike mark tape4
            drive.driveByEncoder(0, 0, 0.5, 1770);  // turn left 180º (only needed to place pixel)
            drive.driveByEncoder(0, 0.3, 0, 785); // moving backward to the spike mark tape
            inDep.changeElevation(-2500); // lower claw
            */

            inDep.changeElevation(2200); // raise claw

            drive.driveByEncoder(0, 0.3, 0, 500); // move backwards
            drive.driveByEncoder(0.3, 0, 0, 869); // strafe left to place pixel correctly
            inDep.changeElevation(-2200); // lower claw
            drive.driveByEncoder(0, -0.3, 0, 500); // move forwards

            drive.driveByEncoder(0, -0.3, 0, 3185); // moving forward to the spike mark tape4
            drive.driveByEncoder(0, 0, -0.5, 1770);  // turn right 180º (only needed to place pixel)
            //drive.driveByEncoder(0.3, 0, 0, 200); // strafe right to finish parking
            drive.driveByEncoder(0, 0.3, 0, 400); // moving backward to the spike mark tape
            drive.driveByEncoder(0, 0.3, 0, 75+75+75); // moving backward to the spike mark tape


        } else if (propLocation == 1) {
            String teeee = "WARNING: PROP MAY GET CAUGHT AFTER BEING PUSHED";
            // center
            inDep.changeElevation(2200); // raise claw
            drive.driveByEncoder(0, 0.3, 0, 300);
            drive.driveByEncoder(0.3, 0, 0, 700); // strafe left to the spike mark tape
            drive.driveByEncoder(0, -0.3, 0, 1400); // move forward to the spike mark tape
//            drive.driveByEncoder(0, -0.3, 0, 1); // brake
            drive.driveByEncoder(0, 0, -0.5, 920);  // turn right
            inDep.changeElevation(-2200); // lower claw
            //drive.driveByEncoder(0.3, 0, 0, 60); // strafe left to the spike mark tape
            drive.driveByEncoder(0, -0.3, 0, 3200); // moving forward to the spike mark tape
            drive.driveByEncoder(0, 0, -0.5, 1800);  // turn right 180º (only needed to place pixel)
            drive.driveByEncoder(0.3, 0, 0, 226.9); // strafe left
            drive.driveByEncoder(0, 0.3, 0, 1000); // continuing backwards to the spike mark tape
            drive.driveByEncoder(0, 0.3, 0, 525); // moving backward to the spike mark tape
        } else if (propLocation == 0) {
            // left
            inDep.changeElevation(2000); // raise claw

            drive.driveByEncoder(-0.3, 0, 0, 1000); // strafe back right
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
