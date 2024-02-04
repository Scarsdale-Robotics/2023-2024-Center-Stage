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

    public static double
        _1step1=30,_1step2=90,_1step3=2.5,
        _2step1a=30,_2step1b=5,_2step2=50,_2step3=160,
        _3step1=10,_3step2a=0,_3step2b=28,_3step3=160,
        _3step4=11,_3step5=40,_3step6=10;

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
                    .sleepFor(150)
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
                    .forward(_1step1)
                    .turnRight(_1step2)
                    .forward(_1step3)
                    .openRightClaw()
                    .sleepFor(500)
                    .build();
            placeYellow = new MovementSequenceBuilder()
                    .backwardLeft(_2step1a, _2step1b)
                    .raiseArm(_2step2, true)
                    .flipElbow()
                    .raiseArm(_2step3)
                    .openLeftClaw()
                    .sleepFor(150)
                    .build();
            park = new MovementSequenceBuilder()
                    .forward(_3step1)
                    .right(_3step2b)
                    .restElbow()
                    .lowerArm(_3step3, true)
                    .backward(_3step4)
                    .lowerArm(_3step5)
                    .lowerArm(_3step6)
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