package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareRobot;
import org.firstinspires.ftc.teamcode.subsystems.RobotSystem;
import org.firstinspires.ftc.teamcode.subsystems.CVSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.InDepSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.movement.MovementSequence;
import org.firstinspires.ftc.teamcode.subsystems.movement.MovementSequenceBuilder;

@Autonomous(name = "Auto Test")
public class AutoTest extends LinearOpMode {
    final private ElapsedTime runtime = new ElapsedTime();
    private InDepSubsystem inDep;
    private DriveSubsystem drive;
    private CVSubsystem cvFront;
    @Override
    // The "Main" code will go in here
    public void runOpMode() {
        RobotSystem robot = new RobotSystem(hardwareMap, false, this, telemetry);
        robot.getInDep().close();
        drive = robot.getDrive();
        runtime.reset();
        inDep.close();

        waitForStart();

        // build a MovementSequence here
        MovementSequence testSequence = new MovementSequenceBuilder()
                .forward(12)
                .backward(12)
                .left(12)
                .right(12)
                .openBothClaws()
                .sleepFor(500)
                .closeBothClaws()
                .sleepFor(500)
                .openRightClaw()
                .sleepFor(500)
                .openLeftClaw()
                .sleepFor(500)
                .closeRightClaw()
                .sleepFor(500)
                .closeLeftClaw()
                .sleepFor(2000)
                .turnRight(90)
                .turnLeft(90)
                .raiseArm(60)
                .lowerArm(60)
                .forwardLeft(10,10)
                .backwardRight(10,10)
                .forwardRight(10,10)
                .backwardLeft(10,10)
                .forward(10)
                .raiseArm(60, true)
                .backward(10)
                .lowerArm(60, true)
                .build();

        telemetry.addData("aaron's mother","");
        telemetry.update();

        //perform the actual movements here in sequence
        drive.followMovementSequence(testSequence);
        drive.stopController();
        sleepFor(690);
    }

    /**
     * Smart sleep with opMode running check.
     * @param ms Timeout in milliseconds.
     */
    private void sleepFor(long ms) {
        runtime.reset();
        while (opModeIsActive() && (runtime.milliseconds() < ms));
    }
}