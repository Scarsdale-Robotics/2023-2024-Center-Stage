package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.teamcode.HardwareRobot;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SleeveDetector;

import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@Autonomous()
public class AutoRight2 extends LinearOpMode {
    double placeSpeed = 0.35, driveSpeed = 0.65, rotateSpeed = 0.2;
    // This code was written by a sleepless Stephen at like 5am
    // I was bored and decided to implement a new Auto strategy
    //
    // In theory, this strategy should be 1+5 solo on high junctions
    // However all of these encoder values are estimated since I
    // obviously do not have access to Murphy at home.
    //
    // For more information please check out the google drawing
    // in the programming folder titled "EXPERIMENTAL 1+5 Auto Right 2
    // Iteration 1 Diagram" where I drew out the strategy.
    //
    // I also cleaned up redundant code & organized/formatted
    // code better e.g. made a new local method "getParkingLocation()"
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
        telemetry.addData("Angle:", (robot.getYaw()+360)%360);
        waitForStart();
        int sleeve = getParkingLocation(sleeveDetector);
        sleeveDetector.close();
        // -----~~~~~========≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡========~~~~~----- //



        // -----~~~~~=====≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡AUTO CODE≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡=====~~~~~----- //
        // strafing 1 tile = 1166 steps ||  for/backward 1 tile = 700 steps ||  turn 90° = 128 steps
        LiftSubsystem.Level high = LiftSubsystem.Level.HIGH;

        // center robot on the third tile from the right
        drive.moveByEncoder(0, placeSpeed, 0, 105, lift, high);
        drive.moveByEncoder(-placeSpeed, 0, 0, 720+584, lift, high);
        // stop movement
        drive.moveByEncoder(placeSpeed, 0, 0, 1, lift, high);

        // turn right 90°

        drive.driveRobotCentric(0, 0, 0);
        drive.driveRobotCentric(0, 0, rotateSpeed);
        while (opModeIsActive() && robot.getYaw() > -90) {
//            telemetry.addData("Angle: ", robot.getYaw());
//            drive.moveByEncoder(placeSpeed, 0, rotateSpeed, 10, lift, high);
        }
        drive.driveRobotCentric(0, 0, 0);
//        drive.moveByEncoder(0, 0, driveSpeed, 129*7-1, lift, high);
//        drive.moveByEncoder(0, 0, -driveSpeed, 1, lift, high);
        // align with far HIGH junction
        drive.moveByEncoder(-placeSpeed, 0, 0, 2915, lift, high);
        // correct for turning
        //drive.moveByEncoder(0, 0, driveSpeed, 128, lift, high);
        // approach & drop cone on junction
        drive.moveByEncoder(0, placeSpeed, 0, 325, lift, high);
        claw.open();

        // approach cone stack
        LiftSubsystem.Level[] stackLevel = {LiftSubsystem.Level.PICKUP,LiftSubsystem.Level.STACK_2,
                LiftSubsystem.Level.STACK_3,LiftSubsystem.Level.STACK_4,LiftSubsystem.Level.STACK_5};
        drive.moveByEncoder(0, -placeSpeed, 0, 150, lift, stackLevel[4]);
        drive.moveByEncoder(driveSpeed, 0, 0, 583, lift, stackLevel[4]);
        drive.moveByEncoder(0, driveSpeed, 0, 1550, lift, stackLevel[4]);

        // the claw is now surrounding the top stack cone
        // begin loop for all 5 cones
        for (int ii = 0; ii < 5; ii++) {
            claw.close();
            // give some time for the lift to rise up so it doesn't knock the cone stack over
            drive.moveByEncoder(0, -0.1, 0, 150, lift, high);
            // approach & drop cone on junction
            drive.moveByEncoder(0, -placeSpeed, 0, 1400, lift, high);
            drive.moveByEncoder(-placeSpeed, 0, 0, 583, lift, high);
            drive.moveByEncoder(0, placeSpeed, 0, 150, lift, high);
            claw.open();

            // return to cone stack
            drive.moveByEncoder(0, -placeSpeed, 0, 150, lift, stackLevel[Math.max(0,3-ii)]);
            drive.moveByEncoder(driveSpeed, 0, 0, 583, lift, stackLevel[Math.max(0,3-ii)]);
            if (ii == 4) break; // stop on third closest, third to right side tile after last cone
            drive.moveByEncoder(0, driveSpeed, 0, 1550, lift, stackLevel[3-ii]);
        }

        // park in CV area
        drive.moveByEncoder(0, driveSpeed, 0,700*sleeve, lift, stackLevel[0]);
        drive.moveByEncoder(driveSpeed, 0, 0, 583, lift, stackLevel[0]);
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
