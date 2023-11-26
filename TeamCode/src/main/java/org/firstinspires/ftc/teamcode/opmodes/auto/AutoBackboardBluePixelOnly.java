package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareRobot;
import org.firstinspires.ftc.teamcode.subsystems.CVSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.InDepSubsystem;

@Autonomous(name = "Auto Backboard Blue Pixel Only")
public class AutoBackboardBluePixelOnly extends LinearOpMode {
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

        // cv or manual location for testing
        boolean[] buttons = new boolean[4];
        while (!(buttons[0] || buttons[1] || buttons[2] || buttons[3])) {
            buttons[0] = gamepad1.triangle;
            buttons[1] = gamepad1.square;
            buttons[2] = gamepad1.x;
            buttons[3] = gamepad1.circle;
        }


        //Start actual Auto now | choose cv or manual prop location
        int propLocation = 0;
        if (buttons[0])
            propLocation = cv.getTeamPropLocation(); // 0 = left, 1 = center, 2 = right
        else if (buttons[1])
            propLocation = 0;
        else if (buttons[2])
            propLocation = 1;
        else if (buttons[3])
            propLocation = 2;


        //only run this when 2 works and vals are updated. may hit truss
        if (propLocation == 0) {
            // left
            drive.driveByEncoder(0, -0.3, 0, 1200); // moving forward toward the pixel placing area
            drive.driveByEncoder(0, 0.3, 0, 1); // brake
            drive.driveByEncoder(0, 0, 0.5, 885);  // turn left
            drive.driveByEncoder(0, -0.3, 0, 100); // moving forward to the spike mark tape
            drive.driveByEncoder(0, 0.3, 0, 1); // brake
            inDep.open(); // open claw to place the pixel
        } else if (propLocation == 1) {
            // center
            drive.driveByEncoder(0, -0.3, 0, 1300); // moving forward toward the pixel placing area
            drive.driveByEncoder(0, 0.3, 0, 1); // brake
            inDep.open(); // open claw to place the pixel
        } else {
            // right
            drive.driveByEncoder(0, -0.3, 0, 1200); // moving forward toward the pixel placing area
            drive.driveByEncoder(0, 0.5, 0, 1); // brake
            drive.driveByEncoder(0, 0, -0.5, 885);  // turn right
            drive.driveByEncoder(0, -0.3, 0, 100); // moving forward to the spike mark tape
            drive.driveByEncoder(0, 0.3, 0, 1); // brake
            inDep.open(); // open claw to place the pixel
        }
        stop();
    }

    public void stop(DriveSubsystem drive) {
        drive.driveByEncoder(0, 0, 0, 0);
    }
}
