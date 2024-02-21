package org.firstinspires.ftc.teamcode.opmodes.calibration;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.RobotSystem;
import org.firstinspires.ftc.teamcode.subsystems.movement.MovementSequence;
import org.firstinspires.ftc.teamcode.subsystems.movement.MovementSequenceBuilder;

@Autonomous
@Config
public class AprilTagAlign extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotSystem robot = new RobotSystem(hardwareMap, false, this, telemetry);
        waitForStart();
        MovementSequence main = new MovementSequenceBuilder()
                .alignWithAprilTagParRot(9, 25)
                .alignWithAprilTagPos(9, 20, -5)
                .build();
        robot.getDrive().followMovementSequence(main);
    }
}
