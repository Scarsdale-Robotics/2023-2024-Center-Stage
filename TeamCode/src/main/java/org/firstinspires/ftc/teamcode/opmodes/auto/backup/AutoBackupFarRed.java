package org.firstinspires.ftc.teamcode.opmodes.auto.backup;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareRobot;
import org.firstinspires.ftc.teamcode.subsystems.CVSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.InDepSubsystem;

@Autonomous(name = "No CV Auto Far Red") //turns first square
public class AutoBackupFarRed extends LinearOpMode {
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

        //Before CV
        drive.driveByEncoder(0, -0.3, 0, 30); // move forward
        //drive.driveByEncoder(0, 0, 0.5, 56); // turn left to counter initial offset kinda but not fully, intentionally
        drive.driveByEncoder(-0.3, 0, 0, 400-25); // strafe right
        //Start actual Auto now | choose cv or manual prop location
        Thread.sleep(1000);

        //CV
        drive.driveByEncoder(0.3, 0, 0, 260-25); // strafe left
        drive.driveByEncoder(0, -0.3, 0, 350); // move forward

        //More CV
        boolean isCloseRed = false;
        int moveOffset = 0;
        drive.driveByEncoder(0, -0.3, 0, 800 + moveOffset); // moving forward toward the pixel placing area
        drive.driveByEncoder(0, 0.5, 0, 1); // brake
        drive.driveByEncoder(0, 0, 0.5, 915 + (isCloseRed ? -27 : 0));  // turn left
        drive.driveByEncoder(0, -0.3 * (isCloseRed ? 1 : -1), 0, (isCloseRed ? 85 : 85)); // moving back/approach
        drive.driveByEncoder(0, -0.3, 0, 10); // brake
        drive.driveByEncoder(-0.3, 0, 0, (isCloseRed ? 0 : 200)); // strafe right

        Thread.sleep(1000); //wait 1 sec for teammate to do auto

        //Rest of Auto
        inDep.changeElevation(2000); // raise claw
        drive.driveByEncoder(-0.3, 0, 0, 1000); // strafe back right
        drive.driveByEncoder(0, 0.3, 0, 3750); // moving backward to the spike mark tape
        inDep.changeElevation(-2000); // lower claw
        //inDep.changeElevation(-1500); // lower claw
        stop();
    }

    public void stop(DriveSubsystem drive) {
        drive.driveByEncoder(0, 0, 0, 0);
    }
}
