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
public class AutoLeft extends LinearOpMode {
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

        drive.moveByEncoder(0, placeSpeed, 0, 70, lift, LiftSubsystem.Level.LOW);
        drive.moveByEncoder(placeSpeed, 0, 0, 385, lift, LiftSubsystem.Level.LOW);
        drive.moveByEncoder(0, placeSpeed, 0, 200, lift, LiftSubsystem.Level.LOW);

        claw.open();


        drive.moveByEncoder(0, -driveSpeed, 0, 200, lift, LiftSubsystem.Level.PICKUP);
        claw.close();
        if (sleeve == 2)
            drive.moveByEncoder(driveSpeed, 0, 0,600, lift, LiftSubsystem.Level.PICKUP);
        else if(sleeve == 1)
            drive.moveByEncoder(-driveSpeed, 0, 0,550, lift, LiftSubsystem.Level.PICKUP);
        else if (sleeve == 0)
            drive.moveByEncoder(-driveSpeed, 0, 0,2000, lift, LiftSubsystem.Level.PICKUP);
        drive.moveByEncoder(0, driveSpeed, 0, 1150, lift, LiftSubsystem.Level.PICKUP);

    }
}
