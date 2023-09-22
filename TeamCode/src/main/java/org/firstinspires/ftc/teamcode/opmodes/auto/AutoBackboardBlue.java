package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareRobot;
import org.firstinspires.ftc.teamcode.subsystems.core.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.CVSubsystem;
//import time;

@Autonomous(name = "Auto Backboard Blue")
public class AutoBackboardBlue extends LinearOpMode {
    @Override
    // The "Main" code will go in here
    public void runOpMode() {
        //HardwareRobot robot = new HardwareRobot(hardwareMap);
        /*DriveSubsystem drive = new DriveSubsystem(
                robot.leftFront,
                robot.rightFront,
                robot.leftBack,
                robot.rightBack,
                robot.imu,
                this
        );*/

        CVSubsystem cvSubsystem = new CVSubsystem();
        waitForStart();
        int tracker = 0;

        while (opModeIsActive()) {
            tracker++;


            //telemetry.addData("I want to die now in: ", 2023);
            telemetry.update();
            //sleep(1000);

            double oe = cvSubsystem.getAprilTagRotationalOffset(9);
            telemetry.addData("I want to answer: ", oe);

//            for (Integer id : cvSubsystem.ids) {
//                telemetry.addData("ID: ", id);
//            }

            // telemetry.addData("Death: ", cvSubsystem.ids.size());
            telemetry.addData("Tracker: ", tracker);
            telemetry.update();

            //sleep(100);
        }


    }
}
