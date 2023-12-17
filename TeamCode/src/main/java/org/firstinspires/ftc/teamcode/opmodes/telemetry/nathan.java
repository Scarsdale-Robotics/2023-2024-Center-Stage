package org.firstinspires.ftc.teamcode.opmodes.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareRobot;
import org.firstinspires.ftc.teamcode.opmodes.SubsystemInitializer;
import org.firstinspires.ftc.teamcode.subsystems.CVSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.InDepSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.movement.MovementThread;

@Autonomous(name = "temp nathan")
public class nathan extends LinearOpMode {

    @Override
    // The "Main" code will go in here
    public void runOpMode() {
        SubsystemInitializer subsystems = new SubsystemInitializer(new HardwareRobot(hardwareMap), false, this, telemetry);

        waitForStart();

        telemetry.addData("team prop loc: ", "---");
        telemetry.update();
        while (opModeIsActive()) {
            int propLocation = subsystems.getCVFront().getPropLocation(); // has telemetry methods inside
            telemetry.update();
        }

    }
}
