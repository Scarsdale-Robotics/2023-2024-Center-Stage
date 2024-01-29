package org.firstinspires.ftc.teamcode.opmodes.calibration;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareRobot;
import org.firstinspires.ftc.teamcode.subsystems.CVSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RobotSystem;

@Autonomous(name = "temp nathan")
public class nathan extends LinearOpMode {

    @Override
    // The "Main" code will go in here
    public void runOpMode() {
        RobotSystem robot = new RobotSystem(hardwareMap, true, this, telemetry);

        waitForStart();

        telemetry.addData("team prop loc: ", "---");
        telemetry.update();
        while (opModeIsActive()) {
            robot.getCv().moveToPixels();
            telemetry.addData("sdjfl;asdf", robot.getCv().getPixelsCenter().x);
            telemetry.addData("sdjfl;asdfy", robot.getCv().getPixelsCenter().y);
            telemetry.update();
        }

    }
}
