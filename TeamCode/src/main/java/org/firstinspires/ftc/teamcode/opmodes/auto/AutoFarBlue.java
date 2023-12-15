package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareRobot;
import org.firstinspires.ftc.teamcode.subsystems.CVSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.InDepSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.movement.MovementSequence;
import org.firstinspires.ftc.teamcode.subsystems.movement.MovementSequenceBuilder;

@Autonomous(name = "Auto Far Blue")
public class AutoFarBlue extends LinearOpMode {
    final private ElapsedTime runtime = new ElapsedTime();
    private HardwareRobot robot;
    private InDepSubsystem inDep;
    private DriveSubsystem drive;
    private CVSubsystem cv;
    @Override
    // The "Main" code will go in here
    public void runOpMode() throws InterruptedException {
        robot = new HardwareRobot(hardwareMap);
        inDep = new InDepSubsystem(
                robot.arm1,
                robot.arm2,
                robot.rightClaw,
                robot.leftClaw,
                robot.wrist,
                robot.elbow,
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
        inDep.closeClaws();
        runtime.reset();

        waitForStart();

        sleepFor(500);

        // Start actual Auto now | cv
        int propLocation = cv.getPropLocation();

        MovementSequence placePurple = new MovementSequenceBuilder().build(),
                moveToBackdrop = new MovementSequenceBuilder().build(),
        placeYellow = new MovementSequenceBuilder().build();

        if (propLocation == 0) {
            // left

            // build the sequence of movements here
            placePurple = new MovementSequenceBuilder()
                    .forward(37.7) // moving forward toward the pixel placing area
                    .turnLeft(90) // turn left 90 degrees
                    .forwardRight(2.5, 4.8) // align with tape
                    .openClaw() // open claw to place the pixel
                    .raiseArm(63) // raise arm
                    .build();
            moveToBackdrop = new MovementSequenceBuilder()
                    .backwardRight(15.4, 17.6) // move to align with truss
                    .lowerArm(63) // lower arm
                    .forward(113.7) // move forwards towards the backstage
                    .turnLeft(183) // turn left 180º (only needed to place pixel)
                    .build();
            placeYellow = new MovementSequenceBuilder()
                    .backwardRight(19.3, 12.3) // moving backward towards the backdrop
                    .raiseArm(120)
                    .openClaw()
                    .lowerArm(120)
                    .closeClaw()
                    .turnLeft(90)
                    .build();

        } else if (propLocation == 1) {
            placePurple = new MovementSequenceBuilder()
                    .forward(24.69) // Moving forward toward the pixel placing area
                    .backward(1.70) // Move backward to not hit pixel on turn
                    .raiseArm(63) // raise arm
                    .build();

            moveToBackdrop = new MovementSequenceBuilder()
                    .backward(9.26) // Move backward
                    .right(11.2) // Strafe right
                    .forward(40.12) // Move forward
                    .turnLeft(89) // Turn left
                    .lowerArm(62.86) // Lower arm
                    .right(1.6) // Strafe right
                    .forward(129.63) // Move forward
                    .turnLeft(180) // Turn left
                    .backward(16.20) // Move backward
                    .build();


        } else if (propLocation == 2) {
            placePurple = new MovementSequenceBuilder()
                    .forward(24.69) // Moving forward toward the pixel placing area
                    .forward(0.03) // Brake
                    .turnRight(85.5) // Turn right
                    .backward(6.48) // Moving back to center
                    .forward(0.31) // Brake
                    .left(3.2) // Strafe left
                    .raiseArm(57.14) // Raise claw
                    .build();

            moveToBackdrop = new MovementSequenceBuilder()
                    .left(16.0) // Strafe left
                    .backward(115.74) // Move backward
                    .lowerArm(57.14) // Lower claw
                    .build();
        }

        // perform the actual movements here in sequence
        drive.followMovementSequence(placePurple);
        drive.followMovementSequence(moveToBackdrop);
        drive.followMovementSequence(placeYellow);

        stopRobot();
    }

    /**
     * Smart sleep with opMode running check.
     * @param ms Timeout in milliseconds.
     */
    private void sleepFor(long ms) {
        runtime.reset();
        while (opModeIsActive() && (runtime.milliseconds() < ms));
    }

    public void stopRobot() {
        drive.driveByEncoder(0, 0, 0, 0);
    }
}