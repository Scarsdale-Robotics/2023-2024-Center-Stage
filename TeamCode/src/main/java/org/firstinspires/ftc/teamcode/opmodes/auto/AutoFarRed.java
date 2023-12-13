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
@Autonomous(name = "Auto Far Red")
public class AutoFarRed extends LinearOpMode {
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

        MovementSequence initCV = new MovementSequenceBuilder()
                .left(3.76) // Strafe left
                .forward(10.80) // Move forward
                .build();

        if (propLocation == 2) { // right
            approachTape = new MovementSequenceBuilder()
                    .forward(21.60) // Moving forward toward the pixel placing area
                    .turnRight(88.8) // Turn right 90 degrees
                    .forward(0.0) // Moving forward (or backward) to the spike mark tape
                    .left(4.8) // Strafe left to place pixel correctly
                    .raiseArm(62.86) // Raise claw
                    .build();

            parkInBackdrop = new MovementSequenceBuilder()
                    .backward(15.43) // Move backward
                    .left(20.0) // Strafe left
                    .lowerArm(62.86) // Lower claw
                    .forward(15.43) // Move forward
                    .forward(98.30) // Continue moving forward
                    .turnRight(177.0) // Turn right
                    .backward(12.35) // Move backward
                    .backward(6.94) // Continue moving backward
                    .build();
        } else if (propLocation == 1) { // center
            approachTape = new MovementSequenceBuilder()
                    .forward(24.04) // Moving forward toward the pixel placing area
                    .raiseArm(62.86) // Raise claw
                    .build();

            parkInBackdrop = new MovementSequenceBuilder()
                    .backward(9.26) // Move backward
                    .left(12.8) // Strafe left
                    .forward(43.21) // Move forward
                    .turnRight(88.0) // Turn right
                    .forward(98.77) // Continue moving forward
                    .turnRight(180.0) // Turn right again
                    .left(3.63) // Strafe left
                    .backward(47.07) // Move backward
                    .build();
        } else if (propLocation == 0) { // left
            approachTape = new MovementSequenceBuilder()
                    .forward(24.69) // Moving forward toward the pixel placing area
                    .forward(0.03) // Brake
                    .turnLeft(86.5) // Turn left
                    .forward(-4.78) // Moving back/approach
                    .forward(0.31) // Brake
                    .right(3.2) // Strafe right
                    .raiseArm(57.14) // Raise claw
                    .build();

            parkInBackdrop = new MovementSequenceBuilder()
                    .right(16.0) // Strafe right
                    .backward(115.74) // Move backward
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