package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareRobot;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SleeveDetector;

@Autonomous()
public class AutoRight extends LinearOpMode {
    @Override
    public void runOpMode() {
        double placeSpeed = 0.25;
        double driveSpeed = 0.5;

        HardwareRobot robot = new HardwareRobot(hardwareMap);

        SleeveDetector sleeveDetector = new SleeveDetector(robot.camera, telemetry);

        DriveSubsystem drive = new DriveSubsystem(
                robot.leftFront,
                robot.rightFront,
                robot.leftBack,
                robot.rightBack,
                robot.imu,
                this
        );

        LiftSubsystem lift = new LiftSubsystem(robot.lift);
        ClawSubsystem claw = new ClawSubsystem(robot.claw);

        claw.close();

        int sleeve = -1;
        ElapsedTime t = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        while(opModeInInit() && sleeve == -1 && t.milliseconds() < 3000) {
            sleeve = sleeveDetector.getParkingLocation();
            telemetry.addLine("Starting camera...");
            telemetry.addData("Claw position: ", claw.servo.getPosition());
            telemetry.update();
        }
        telemetry.addData("Pre Parking: ", sleeve);

        if(sleeve==-1){
            sleeve=0;
            telemetry.addData("CV ERROR", null);
        }
        telemetry.update();

        waitForStart();

        //get sleeve via CV
        sleeve = -1;
        t.reset();
        while(opModeIsActive() && sleeve == -1 && t.milliseconds() < 500) {
            sleeve = sleeveDetector.getParkingLocation();
            telemetry.addData("Pre Parking: ", sleeve);
            telemetry.update();
        }
        if(sleeve==-1){
            sleeve=0;
            telemetry.addData("CV ERROR", null);
            telemetry.update();
        }

        sleeveDetector.close();
        // -----~~~~~===============~~~~~----- //


        // -----~~~~~=====AUTO CODE=====~~~~~----- //
        // tuning
        int startStrafeDisplacement = 320;
        // move forward and don't hit wall
        // raise lift to LOW junction
        drive.moveByEncoder(0, placeSpeed, 0, 70, lift, LiftSubsystem.Level.LOW);
        // align with junction
        drive.moveByEncoder(-placeSpeed, 0, 0, 400 + startStrafeDisplacement, lift, LiftSubsystem.Level.LOW);
        // move forward and approach junction
        drive.moveByEncoder(0, placeSpeed, 0, 200, lift, LiftSubsystem.Level.LOW);

        // drop cone on junction
        claw.open();

        // align with back wall
        drive.moveByEncoder(0, -driveSpeed, 0, 200, lift, LiftSubsystem.Level.PICKUP);

        // close claw
        claw.close();
        //int parkRightTest = 200;
        // park given CV value

        if (sleeve == 2) { // right zone
            drive.moveByEncoder(0, 0, driveSpeed, 10, lift, LiftSubsystem.Level.PICKUP);
            drive.moveByEncoder(driveSpeed, 0, 0,1550 + 200 , lift, LiftSubsystem.Level.PICKUP);
        }
        else if(sleeve == 1) // middle zone
            drive.moveByEncoder(driveSpeed, 0, 0,350 + 200 , lift, LiftSubsystem.Level.PICKUP);
        else if (sleeve == 0) // left zone
            drive.moveByEncoder(-driveSpeed, 0, 0,550 , lift, LiftSubsystem.Level.PICKUP);

        // enter zone by moving forward
        drive.moveByEncoder(0, driveSpeed, 0, 1050, lift, LiftSubsystem.Level.PICKUP);
        // -----~~~~~========≡≡≡≡≡≡≡≡≡≡========~~~~~----- //
    }
}
