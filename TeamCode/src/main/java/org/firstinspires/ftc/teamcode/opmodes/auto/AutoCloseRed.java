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

@Autonomous(name = "Auto Close Red")
public class AutoCloseRed extends LinearOpMode {
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
                true,
                this);
        inDep.close();
        runtime.reset();

        waitForStart();

        sleepFor(500);

        // Start actual Auto now | cv
        int propLocation = cv.getPropLocation();

        MovementSequence approachTape = new MovementSequenceBuilder().build(),
                parkInBackdrop = new MovementSequenceBuilder().build();

        if (propLocation == 2) { // right
            approachTape = new MovementSequenceBuilder()
                    .forward(24.69) // Moving forward toward the pixel placing area
                    .forward(0.03) // Brake
                    .turnRight(85.8) // Turn right 90 degrees
                    .backward(4.63) // Moving forward (or backward) to the spike mark tape
                    .forward(0.03) // Brake
                    .left(0.0) // Strafe left to place pixel correctly
                    .raiseArm(57.14) // Raise claw
                    .build();

            parkInBackdrop = new MovementSequenceBuilder()
                    .right(19.2) // Strafe right
                    .forward(30.86) // Move forward
                    .turnLeft(185.5) // Turn left
                    .backward(16.98) // Move backward
                    .closeClaw() // Close claw
                    .lowerArm(57.14) // Lower claw
                    .build();
        } else if (propLocation == 1) { // center
            approachTape = new MovementSequenceBuilder()
                    .forward(20.67) // Moving forward toward the pixel placing area
                    .raiseArm(57.14) // Raise claw
                    .build();

            parkInBackdrop = new MovementSequenceBuilder()
                    .backward(32.41) // Move backward
                    .turnLeft(87.8) // Turn left
                    .backward(43.21) // Move backward
                    .closeClaw() // Close claw
                    .lowerArm(57.14) // Lower claw
                    .build();
        } else if (propLocation == 0) { // left
            approachTape = new MovementSequenceBuilder()
                    .forward(24.69) // Moving forward toward the pixel placing area
                    .turnLeft(88.8) // Turn left
                    .forward(2.47) // Moving back/approach
                    .forward(0.31) // Brake
                    .right(0.0) // Strafe right
                    .raiseArm(57.14) // Raise claw
                    .build();

            parkInBackdrop = new MovementSequenceBuilder()
                    .backward(7.72) // Move backward
                    .left(22.4) // Strafe left
                    .backward(37.04) // Move backward
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