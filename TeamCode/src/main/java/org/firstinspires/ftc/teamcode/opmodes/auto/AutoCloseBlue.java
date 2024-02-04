package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.RobotSystem;
import org.firstinspires.ftc.teamcode.subsystems.CVSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.InDepSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.movement.MovementSequence;
import org.firstinspires.ftc.teamcode.subsystems.movement.MovementSequenceBuilder;

@Autonomous(name = "Auto Close Blue")
@Config
public class AutoCloseBlue extends LinearOpMode {
    final private ElapsedTime runtime = new ElapsedTime();
    private InDepSubsystem inDep;
    private DriveSubsystem drive;
    private CVSubsystem cvFront;
    private CVSubsystem cvBack;

    @Override
    // The "Main" code will go in here
    public void runOpMode() {
        RobotSystem robot = new RobotSystem(hardwareMap, false, this, telemetry);
        robot.getInDep().close();
        drive = robot.getDrive();
        runtime.reset();

        waitForStart();

        // Start actual Auto now | cv
        MovementSequence initCV = new MovementSequenceBuilder()
                //stuff
                .build();
        drive.followMovementSequence(initCV);
        int propLocation = robot.getCv().getPropLocation();
//        int propLocation = 0;

        MovementSequence placePurple = new MovementSequenceBuilder().build(),
                approachWhite = new MovementSequenceBuilder().build(),
                placeWhite = new MovementSequenceBuilder().build(),
                placeYellow = new MovementSequenceBuilder().build(),
                park = new MovementSequenceBuilder().build();

        int WHITE_REPS = 2;
        double WHITE_PX_HEIGHT = 22;

        if (propLocation == 0) {
            placePurple = new MovementSequenceBuilder()
                    .closeBothClaws()
                    .forwardLeft(9+10.80, 7+4.16)
                    .openRightClaw() // drop purple pixel
                    .sleepFor(500)
                    .backward(5)
                    .build();
            placeYellow = new MovementSequenceBuilder()
                    .raiseArm(30)
                    .turnLeft(90, true)
                    .forwardRight(24,10)
                    .openLeftClaw()
                    .sleepFor(150)
                    .build();
            park = new MovementSequenceBuilder()
                    .backward(5)
                    .turnRight(180)
                    .right(20)
                    .backward(9)
                    .lowerArm(20)
                    .lowerArm(10)
                    .forward(2, true)
                    .build();

        } else if (propLocation == 1) {
            placePurple = new MovementSequenceBuilder()
                    .closeBothClaws()
                    .forward(26.5)
                    .openRightClaw()
                    .sleepFor(500)
                    .build();
            placeYellow = new MovementSequenceBuilder()
                    .backward(5)
                    .raiseArm(30)
                    .turnLeft(90)
                    .forwardRight(33, 10.69420)
                    .openLeftClaw()
                    .sleepFor(300)
                    .build();
            park = new MovementSequenceBuilder()
                    .backward(5)
                    .backwardLeft(5, 26)
                    .turnRight(180)
                    .backward(16)
                    .lowerArm(20)
                    .lowerArm(10)
                    .forward(2, true)
                    .build();

        } else if (propLocation == 2) {
            placePurple = new MovementSequenceBuilder()
                    .closeBothClaws()
                    .forward(30)
                    .turnRight(90)
                    .forward(2.5)
                    .openRightClaw()
                    .sleepFor(500)
                    .build();
            placeYellow = new MovementSequenceBuilder()
                    .backwardLeft(28, 7)
                    .raiseArm(50, true)
                    .flipElbow()
                    .raiseArm(170)
                    .openLeftClaw()
                    .sleepFor(150)
                    .build();
            park = new MovementSequenceBuilder()
                    .forward(10)
                    .right(28)
                    .restElbow()
                    .lowerArm(170, true)
                    .backward(17)
                    .lowerArm(40)
                    .lowerArm(10)
                    .forward(2, true)
                    .build();
        }

        // perform the actual movements here in sequence
        drive.followMovementSequence(placePurple);
        drive.followMovementSequence(placeYellow);
//        for (int i=0;i<WHITE_REPS;i++)
//        {
//            drive.followMovementSequence(approachWhite);
//            drive.followMovementSequence(placeWhite);
//        }
        drive.followMovementSequence(park);

        drive.stopController();
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