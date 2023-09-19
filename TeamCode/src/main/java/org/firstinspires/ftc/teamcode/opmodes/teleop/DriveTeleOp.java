package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareRobot;
import org.firstinspires.ftc.teamcode.subsystems.core.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.core.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.core.LiftSubsystem;

@TeleOp(name = "Drive TeleOp")
public class DriveTeleOp extends LinearOpMode {
    private double fineLiftControlSpeedDown = 0.3;
    private double fineLiftControlSpeedUp = fineLiftControlSpeedDown + 0.3;

    private double fastDriveCoef = 0.45;
    private double slowDriveCoef = 0.25;

    private double fastStrafeCoef = 1;
    private double slowStafeCoef = 0.75;

    private double driveCoef = fastDriveCoef;
    private double strafeCoef = fastStrafeCoef;

    private double slowTurnCoef = 0.4;
    private double fastTurnCoef = 0.8;

    private double turnCoef = fastTurnCoef;

    private LiftSubsystem.Level storedLevel = LiftSubsystem.Level.HIGH;

    @Override
    public void runOpMode() {
        HardwareRobot robot = new HardwareRobot(hardwareMap);
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

        lift.setLevel(LiftSubsystem.Level.PICKUP);

        waitForStart();

        GamepadEx driverOp1 = new GamepadEx(gamepad1);

        boolean open = false, liftToggled = false, clawToggled = false, slowModeButtonToggled = false, slowModeLiftActive = false, slowModeButtonActive = false;

        boolean liftMoveAtSpeedPrevious = false;

        while (opModeIsActive()) {
            double driveX = 0;
            double driveY = 0;

            ///////////////////
            // Drive Control //
            ///////////////////

            // Binds movement to just the four cardinal directions
            if (Math.abs(driverOp1.getLeftX()) > 0.6) {
                driveX = Math.signum(driverOp1.getLeftX());
                driveY = 0;
            }
            else if (Math.abs(driverOp1.getLeftY()) > 0.6) {
                driveX = 0;
                driveY = Math.signum(driverOp1.getLeftY());
            }else{
                driveX = driverOp1.getLeftX();
                driveY = driverOp1.getLeftY();
            }

            // Drive robot centric
            drive.driveFieldCentric(driveX * strafeCoef, driveY * driveCoef, driverOp1.getRightX() * turnCoef);

            telemetry.addData("DriveX:", driveX);
            telemetry.addData("DriveY:", driveY);


            //////////////////
            // Lift Control //
            //////////////////

            // raise lift one level
            if (gamepad1.b && !liftToggled) {
                liftToggled = true;
                lift.nextLevelUp();
                storedLevel = lift.getLevel();
            }

            // lower lift one level
            if (gamepad1.a && !liftToggled) {
                liftToggled = true;
                lift.nextLevelDown();
                storedLevel = lift.getLevel();
            }

            if (!gamepad1.a && !gamepad1.b) liftToggled = false;

            // toggle slow mode
            if (!gamepad1.x) slowModeButtonToggled = false;
            if (gamepad1.x && !slowModeButtonToggled) {
                slowModeButtonToggled = true;
                slowModeButtonActive = !slowModeButtonActive;
            }

            // If the lift is raised, drive slower for alignment
            if (lift.getLevel() != LiftSubsystem.Level.PICKUP) {
                slowModeLiftActive = true;
            }else{
                slowModeLiftActive = false;
            }

            if (slowModeButtonActive || slowModeLiftActive) {
                driveCoef = slowDriveCoef;
                strafeCoef = slowStafeCoef;
                turnCoef = slowTurnCoef;
            } else {
                driveCoef = fastDriveCoef;
                strafeCoef = fastStrafeCoef;
                turnCoef = fastTurnCoef;
            }

            // quick lower and release
            if (gamepad1.left_bumper) {
                lift.setLevel(LiftSubsystem.Level.PICKUP);
                claw.open();
                open = true;
            } else if (gamepad1.right_bumper) { //quick raise
                lift.setLevel(storedLevel);
            }

            // Resets lift
            if (gamepad2.a && gamepad2.b) {
                lift.resetEncoder();
                gamepad1.rumble(500);
                gamepad2.rumble(500);
            }

            double totalChange = (gamepad1.right_trigger - gamepad1.left_trigger);
            if (totalChange > 0) totalChange*= fineLiftControlSpeedUp;
            else totalChange *= fineLiftControlSpeedDown;

            if (gamepad1.dpad_up) totalChange = 1;
            else if (gamepad1.dpad_down) totalChange = -1;

            if (Math.abs(totalChange) > 0.1) {
                lift.setMode(Motor.RunMode.RawPower);
                lift.runLift(totalChange);
                liftMoveAtSpeedPrevious = true;
            } else {
                if (liftMoveAtSpeedPrevious) {
                    liftMoveAtSpeedPrevious = false;
                    lift.setMode(Motor.RunMode.PositionControl);
                    lift.setTargetToCurrent();
                }
                lift.runLift();
            }


            //////////////////
            // Claw Control //
            //////////////////

            if (!gamepad1.y) clawToggled = false;
            if (gamepad1.y && !clawToggled) {
                if (!open) {
                    claw.open();
                } else {
                    claw.close();
                }
                open = !open;
                clawToggled = true;
            }

            ///////////////////////
            // Telemetry Logging //
            ///////////////////////


            telemetry.addData("slow mode:", slowModeButtonActive || slowModeLiftActive);
            telemetry.addData("pos: ", lift.getPosition());
            telemetry.addData("current level: ", lift.getLevel());
            telemetry.addData("stored level: ", storedLevel);

            telemetry.addData("target: ", lift.atTarget());

            telemetry.addData("claw: ", open);
            telemetry.update();
        }
    }
}
