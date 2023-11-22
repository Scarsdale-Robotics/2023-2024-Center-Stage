package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareRobot;
import org.firstinspires.ftc.teamcode.subsystems.CVSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.InDepSubsystem;

@Autonomous(name = "Auto Backboard Blue")
public class AutoBackboardBlue extends LinearOpMode {
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

        inDep.close();

        waitForStart();

        //Start actual Auto now | choose cv or manual prop location
        int propLocation = -1;
        while ((propLocation < 0 || propLocation > 2) && opModeIsActive()) propLocation = cv.getTeamPropLocation();

        //only run this when 2 works and vals are updated. may hit truss
        if (propLocation == 0) {
            // left
            drive.driveByEncoder(0, -0.3, 0, 1200); // moving forward toward the pixel placing area
            drive.driveByEncoder(0, 0.3, 0, 1); // brake
            drive.driveByEncoder(0, 0, 0.5, 885);  // turn left
            drive.driveByEncoder(0, -0.3, 0, 100); // moving forward to the spike mark tape
            drive.driveByEncoder(0, 0.3, 0, 1); // brake
            inDep.open(); // open claw to place the pixel
            inDep.changeElevation(2500); // raise claw
            drive.driveByEncoder(0, -0.3, 0, 3550); // moving forward to the spike mark tape
            inDep.changeElevation(-2500); // lower claw
        } else if (propLocation == 1) {
            // center
            drive.driveByEncoder(0, -0.3, 0, 1300); // moving forward toward the pixel placing area
            drive.driveByEncoder(0, 0.3, 0, 1); // brake
            inDep.open(); // open claw to place the pixel
            inDep.changeElevation(2500); // raise claw
            drive.driveByEncoder(0, 0.3, 0, 15); // move backwards to the spike mark tape
//            drive.driveByEncoder(0, -0.3, 0, 1); // brake
            drive.driveByEncoder(0, 0, 0.5, 835);  // turn left
            inDep.changeElevation(-2500); // lower claw
            drive.driveByEncoder(-0.3, 0, 0, 75); // move backwards to the spike mark tape
            drive.driveByEncoder(0, -0.3, 0, 3650); // moving forward to the spike mark tape
        } else if (propLocation == 2) {
            // right
            drive.driveByEncoder(0, -0.3, 0, 1300); // moving forward toward the pixel placing area
            drive.driveByEncoder(0, 0.5, 0, 1); // brake
            drive.driveByEncoder(0, 0, -0.5, 885);  // turn right
            //drive.driveByEncoder(0, -0.3, 0, 100); // moving forward to the spike mark tape
            //drive.driveByEncoder(0, 0.3, 0, 1); // brake
            drive.driveByEncoder(0, 0.3, 0, 125); // moving back to center
            drive.driveByEncoder(0, -0.3, 0, 1); // brake
            inDep.open(); // open claw to place the pixel
            //drive.driveByEncoder(0, 0, 0.5, 1770);  // turn left 180º (only needed to place pixel)
            drive.driveByEncoder(0, 0.3, 0, 3750); // moving backward to the spike mark tape
            //inDep.changeElevation(-1500); // lower claw
        }
        stop();
    }

    public void stop(DriveSubsystem drive) {
        drive.driveByEncoder(0, 0, 0, 0);
    }
}
