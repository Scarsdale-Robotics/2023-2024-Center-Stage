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

@Autonomous(name = "Auto Close Blue")
public class AutoCloseBlue extends LinearOpMode {
    final private ElapsedTime runtime = new ElapsedTime();
    private HardwareRobot robot;
    private InDepSubsystem inDep;
    private DriveSubsystem drive;
    private CVSubsystem cv;
    @Override
    // The "Main" code will go in here
    public void runOpMode() {
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
        inDep.close();
        runtime.reset();

        waitForStart();

        sleepFor(500);

        // Start actual Auto now | cv
        int propLocation = cv.getPropLocation();

        MovementSequence approachTape = new MovementSequenceBuilder().build(),
                parkInBackdrop = new MovementSequenceBuilder().build();

        MovementSequence initCV = new MovementSequenceBuilder()
                .left(4.16) // Strafe left
                .forward(10.80) // Move forward
                .build();

        if (propLocation == 0) {
            // left

            // build the sequence of movements here
            approachTape = new MovementSequenceBuilder()
                    .forward(22.22) // Moving forward toward the pixel placing area
                    .forward(0.03) // Brake
                    .turnLeft(84.3) // Turn left 90 degrees
                    .backward(3.09) // Moving forward (or backward) to the spike mark tape
                    .forward(0.03) // Brake
                    .right(4.0) // Strafe right to place pixel correctly
                    .build();

            parkInBackdrop = new MovementSequenceBuilder()
                    .raiseArm(57.14) // Raise claw
                    .left(14.4) // Strafe left
                    .forward(34.29) // Move forward
                    .turnRight(180) // Turn right
                    .backward(20.06) // Move backward
                    .closeClaw() // Close claw
                    .lowerArm(57.14) // Lower claw
                    .build();


        } else if (propLocation == 1) {
            approachTape = new MovementSequenceBuilder()
                    .forward(25.77) // Moving forward toward the pixel placing area
                    .forward(0.03) // Brake
                    .backward(1.70) // Move backward to not hit pixel on turn
                    .build();

            parkInBackdrop = new MovementSequenceBuilder()
                    .raiseArm(57.14) // Raise claw
                    .turnLeft(80) // Turn left
                    .left(14.4) // Strafe left
                    .forward(34.29) // Move forward
                    .turnRight(180) // Turn right
                    .backward(16.98) // Move backward
                    .closeClaw() // Close claw
                    .lowerArm(57.14) // Lower claw
                    .build();


        } else if (propLocation == 2) {
            approachTape = new MovementSequenceBuilder()
                    .forward(24.69) // Moving forward toward the pixel placing area
                    .forward(0.03) // Brake
                    .turnRight(87.0) // Turn right
                    .backward(0.31) // Brake
                    .left(3.2) // Strafe left
                    .build();

            parkInBackdrop = new MovementSequenceBuilder()
                    .raiseArm(57.14) // Raise claw
                    .backward(6.85) // Move backward
                    .right(20.4) // Strafe right
                    .backward(43.21) // Move backward
                    .closeClaw() // Close claw
                    .lowerArm(57.14) // Lower claw
                    .build();
        }

        // perform the actual movements here in sequence
        drive.followMovementSequence(approachTape);
        drive.followMovementSequence(parkInBackdrop);

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