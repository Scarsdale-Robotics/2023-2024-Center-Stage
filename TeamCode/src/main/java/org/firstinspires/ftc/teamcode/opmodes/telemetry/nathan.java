package org.firstinspires.ftc.teamcode.opmodes.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareRobot;
import org.firstinspires.ftc.teamcode.subsystems.RobotSystem;

@Autonomous(name = "temp nathan")
public class nathan extends LinearOpMode {

    @Override
    // The "Main" code will go in here
    public void runOpMode() {
        RobotSystem robot = new RobotSystem(hardwareMap, false, this, telemetry);

        waitForStart();

        telemetry.addData("team prop loc: ", "---");
        telemetry.update();
        while (opModeIsActive()) {
            int propLocation = robot.getCVFront().getPropLocation(); // has telemetry methods inside
            telemetry.update();
        }

    }
}
