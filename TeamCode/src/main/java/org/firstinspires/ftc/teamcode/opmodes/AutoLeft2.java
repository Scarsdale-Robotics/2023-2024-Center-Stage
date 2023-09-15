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
public class AutoLeft2 extends LinearOpMode {
    double placeSpeed = 0.25, driveSpeed = 0.5;
    //
    // Code mirrored from AutoRight2.java by Kenneth
    //
    @Override
    public void runOpMode() {
        // init robot
        HardwareRobot robot = new HardwareRobot(hardwareMap);
        SleeveDetector sleeveDetector = new SleeveDetector(robot.camera, telemetry);
        LiftSubsystem lift = new LiftSubsystem(robot.lift);
        ClawSubsystem claw = new ClawSubsystem(robot.claw);
        DriveSubsystem drive = new DriveSubsystem(
                robot.leftFront,
                robot.rightFront,
                robot.leftBack,
                robot.rightBack,
                robot.imu,
                this
        );

        // get sleeve via CV
        getParkingLocation(sleeveDetector);
        claw.close();
        waitForStart();
        int sleeve = getParkingLocation(sleeveDetector);
        sleeveDetector.close();
        // -----~~~~~========≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡========~~~~~----- //



        // -----~~~~~=====≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡AUTO CODE≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡=====~~~~~----- //
        // strafing 1 tile = 1166 steps ||  for/backward 1 tile = 700 steps ||  turn 90° = 128 steps
        LiftSubsystem.Level high = LiftSubsystem.Level.HIGH;

        // center robot on the third tile from the left
        drive.moveByEncoder(0, placeSpeed, 0, 105, lift, high);
        drive.moveByEncoder(placeSpeed, 0, 0, 2*1166-1304, lift, high);
        // stop movement
        drive.moveByEncoder(-placeSpeed, 0, 0, 1, lift, high);

        // turn left 90°
        drive.moveByEncoder(0, 0, -driveSpeed, 129, lift, high);
        drive.moveByEncoder(0, 0, driveSpeed, 1, lift, high);
        // align with far HIGH junction by strafing right
        drive.moveByEncoder(placeSpeed, 0, 0, 2915, lift, high);
        // correct for turning
        drive.moveByEncoder(0, 0, -driveSpeed, 30, lift, high);
        // approach & drop cone on junction
        drive.moveByEncoder(0, placeSpeed, 0, 325, lift, high);
        claw.open();

        // approach cone stack
        LiftSubsystem.Level[] stackLevel = {LiftSubsystem.Level.PICKUP,LiftSubsystem.Level.STACK_2,
                LiftSubsystem.Level.STACK_3,LiftSubsystem.Level.STACK_4,LiftSubsystem.Level.STACK_5};
        drive.moveByEncoder(0, -placeSpeed, 0, 150, lift, stackLevel[4]); // move back
        drive.moveByEncoder(-driveSpeed, 0, 0, 583, lift, stackLevel[4]); // go left
        drive.moveByEncoder(0, driveSpeed, 0, 1550, lift, stackLevel[4]); // forward to stack

        // the claw is now surrounding the top stack cone
        // begin loop for all 5 cones
        for (int ii = 0; ii < 5; ii++) {
            claw.close();
            // give some time for the lift to rise up so it doesn't knock the cone stack over
            drive.moveByEncoder(0, -0.1, 0, 150, lift, high);
            // approach & drop cone on junction
            drive.moveByEncoder(0, -placeSpeed, 0, 1400, lift, high);
            drive.moveByEncoder(placeSpeed, 0, 0, 583, lift, high);
            drive.moveByEncoder(0, placeSpeed, 0, 150, lift, high);
            claw.open();

            // return to cone stack
            drive.moveByEncoder(0, -placeSpeed, 0, 150, lift, stackLevel[Math.max(0,3-ii)]);
            drive.moveByEncoder(-driveSpeed, 0, 0, 583, lift, stackLevel[Math.max(0,3-ii)]);
            if (ii == 4) break; // stop on third closest, third to left side tile after last cone
            drive.moveByEncoder(0, driveSpeed, 0, 1550, lift, stackLevel[3-ii]);
        }

        // park in CV area
        drive.moveByEncoder(0, driveSpeed, 0,700*(2-sleeve), lift, stackLevel[0]);
        drive.moveByEncoder(-driveSpeed, 0, 0, 583, lift, stackLevel[0]);
        claw.close();

        // -----~~~~~========≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡========~~~~~----- //


    }

    int getParkingLocation(SleeveDetector sleeveDetector) {
        // get sleeve via CV
        int slv = -1;
        ElapsedTime t = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        while((isStarted() ? opModeIsActive() : opModeInInit()) && slv == -1 && t.milliseconds() < (isStarted() ? 500 : 3000)) {
            slv = sleeveDetector.getParkingLocation();
            if (isStarted()) telemetry.addData("Pre Parking: ", slv); else telemetry.addLine("Starting camera...");
            telemetry.update();
        }
        telemetry.addData("Pre Parking: ", slv);

        // check for an error
        if (slv==-1) telemetry.addData("CV ERROR", null);
        slv = slv==-1 ? 0 : slv;
        telemetry.update();

        return slv;
    }
}