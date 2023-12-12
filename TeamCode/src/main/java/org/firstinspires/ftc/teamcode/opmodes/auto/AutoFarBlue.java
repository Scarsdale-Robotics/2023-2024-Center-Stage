package org.firstinspires.ftc.teamcode.opmodes.auto;

import android.service.quicksettings.Tile;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareRobot;
import org.firstinspires.ftc.teamcode.subsystems.CVSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem.*;
import org.firstinspires.ftc.teamcode.subsystems.InDepSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.movement.MovementSequence;
import org.firstinspires.ftc.teamcode.subsystems.movement.MovementSequenceBuilder;

@Autonomous(name = "Auto Far Blue")
public class AutoFarBlue extends LinearOpMode {
    private HardwareRobot robot;
    private InDepSubsystem inDep;
    private DriveSubsystem drive;
    private CVSubsystem cv;
    @Override
    // The "Main" code will go in here
    public void runOpMode() throws InterruptedException {
        robot = new HardwareRobot(hardwareMap);
        inDep = new InDepSubsystem(
                robot.arm,
                robot.claw,
                robot.wrist,
                this,
                telemetry
        );
        drive = new DriveSubsystem(
                robot.leftFront,
                robot.rightFront,
                robot.leftBack,
                robot.rightBack,
                robot.imu,
                inDep,
                this
        );
        cv = new CVSubsystem(robot.camera,
                robot.cameraName,
                drive,
                telemetry,
                false,
                this);
        inDep.close();

        waitForStart();

        Thread.sleep(500);

        // Start actual Auto now | cv
        int propLocation = cv.getPropLocation();

        MovementSequence approachTape = new MovementSequenceBuilder().build(),
                parkInBackdrop = new MovementSequenceBuilder().build();

        if (propLocation == 0) {
            // left

            // build the sequence of movements here
            approachTape = new MovementSequenceBuilder()
                    .forward(37.7) // moving forward toward the pixel placing area
                    .turnLeft(90) // turn left 90 degrees
                    .forward(2.5) // moving forward to the spike mark tape
                    .right(4.8) // strafe right to place pixel correctly
                    .openClaw() // open claw to place the pixel
                    .raiseArm(63) // raise arm
                    .build();
            parkInBackdrop = new MovementSequenceBuilder()
                    .backward(15.4) // move backwards
                    .right(17.6) // strafe right
                    .lowerArm(63) // lower arm
                    .forward(113.7) // move forwards towards the backstage
                    .turnLeft(183) // turn left 180º (only needed to place pixel)
                    .backward(19.3) // moving backward towards the backstage
                    .build();

        } else if (propLocation == 1) {
            // center

        } else if (propLocation == 2) {
            // right

        }

        // perform the actual movements here in sequence
        drive.followMovementSequence(approachTape);
        drive.followMovementSequence(parkInBackdrop);

        stopRobot();
    }

    public void stopRobot() {
        drive.driveByEncoder(0, 0, 0, 0);
    }
}