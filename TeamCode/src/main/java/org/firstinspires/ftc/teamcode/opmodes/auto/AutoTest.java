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

@Autonomous(name = "Auto Test")
public class AutoTest extends LinearOpMode {
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
                false,
                this);
        inDep.close();
        runtime.reset();

        waitForStart();

        // build a MovementSequence here
        MovementSequence testSequence = new MovementSequenceBuilder()
                .forward(12)
                .backward(12)
                .left(12)
                .right(12)
                .openClaw()
                .sleepFor(500)
                .closeClaw()
                .sleepFor(5000)
                .openClaw()
                .sleepFor(500)
                .closeClaw()
                .turnRight(90)
                .turnLeft(90)
                .raiseArm(60)
                .lowerArm(60)
                .build();

        telemetry.addData("aaron's mother","");
        telemetry.update();

//         perform the actual movements here in sequence
        drive.followMovementSequence(testSequence);
        sleepFor(690);
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