package org.firstinspires.ftc.teamcode.opmodes.auto.backup;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareRobot;
import org.firstinspires.ftc.teamcode.subsystems.CVSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.InDepSubsystem;

@Autonomous(name = "No CV Auto Close Blue") //turns first square
public class AutoBackupCloseBlue extends LinearOpMode {
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
        waitForStart();

        //The Code below is technically before CV
        drive.driveByEncoder(0, -0.3, 0, 30); // move forward
        //drive.driveByEncoder(0, 0, 0.5, 56); // turn left to counter initial offset kinda but not fully, intentionally
        drive.driveByEncoder(-0.3, 0, 0, 400); // strafe right

        //The Code below is technically CV
        drive.driveByEncoder(0.3, 0, 0, 260); // strafe left
        drive.driveByEncoder(0, -0.3, 0, 350); // move forward
        //Farther into CV
        drive.driveByEncoder(0, -0.3, 0, 720); // moving forward toward the pixel placing area
        drive.driveByEncoder(0, 0.3, 0, 1); // brake
        drive.driveByEncoder(0, 0, 0.5, 888 -50);  // turn left 90 degrees
        drive.driveByEncoder(0, -0.3 * -1, 0, 50 + 50); // moving forward (or backward) to the spike mark tape
        drive.driveByEncoder(0, 0.3, 0, 1); // brake
        drive.driveByEncoder(-0.3, 0, 0, 300 -50); // strafe right to place pixel correctly

        Thread.sleep(1000); //wait 1 sec for teammate to do auto

        //Regular Auto things

        inDep.changeElevation(2000);
        drive.driveByEncoder(0.3, 0, 0, 900); // move left
        drive.driveByEncoder(0, -0.3, 0, 1111); // move forward a bit
        drive.driveByEncoder(0, 0, -0.5, 1800); // perform 180
        drive.driveByEncoder(0, 0.3, 0, 650); // move backwards to park
        inDep.close();

        inDep.changeElevation(-2000);
        stop();
    }

    public void stop(DriveSubsystem drive) {
        drive.driveByEncoder(0, 0, 0, 0);
    }
}
