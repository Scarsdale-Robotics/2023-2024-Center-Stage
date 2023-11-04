package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.checkerframework.checker.units.qual.Speed;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.HardwareRobot;
import org.firstinspires.ftc.teamcode.SpeedCoefficients;
import org.firstinspires.ftc.teamcode.subsystems.InDepSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.CVSubsystem;

@TeleOp(name = "omg the funny")
public class Clash extends LinearOpMode {
    @Override
    public void runOpMode() {
        // init robot
        HardwareRobot robot = new HardwareRobot(hardwareMap);
        SpeedCoefficients.setMode(SpeedCoefficients.MoveMode.MODE_SLOW);
        DriveSubsystem drive = new DriveSubsystem(
                robot.leftFront,
                robot.rightFront,
                robot.leftBack,
                robot.rightBack,
                robot.imu,
                this
        );
        InDepSubsystem inDep = new InDepSubsystem(
                robot.arm,
                robot.claw,
                robot.wrist,
                this,
                telemetry
        );
        CVSubsystem cv = new CVSubsystem(
                robot.camera,
                drive
        );

        int clashStartup = hardwareMap.appContext.getResources().getIdentifier("clashroyaleintro", "raw", hardwareMap.appContext.getPackageName());
        int heheha = hardwareMap.appContext.getResources().getIdentifier("heheha", "raw", hardwareMap.appContext.getPackageName());
        int pacer = hardwareMap.appContext.getResources().getIdentifier("pacertest", "raw", hardwareMap.appContext.getPackageName());

        SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, clashStartup);
        SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, pacer);

        waitForStart();


        // main TeleOp loop
        boolean soundToggled = false;
        boolean wristToggled = false, clawToggled = false;
        boolean omniMode = false;
        int prevArmEncoder = 0;

        int moving = 1;
        boolean override = false;

        while (opModeIsActive()) {
            //////////////////////////
            // Basic Motion Control //
            //////////////////////////

            // these drive variables are for possible omni mode in the future
            double driveX = 0;
            double driveY = 0;

            if (gamepad1.dpad_up || gamepad2.dpad_up) {
                if (gamepad2.dpad_up) {
                    gamepad1.rumble(500);
                }
                SpeedCoefficients.setMode(SpeedCoefficients.MoveMode.MODE_FAST);
            } else if (gamepad1.dpad_down || gamepad2.dpad_down) {
                if (gamepad2.dpad_down) {
                    gamepad1.rumble(500);
                }
                SpeedCoefficients.setMode(SpeedCoefficients.MoveMode.MODE_SLOW);
            }


            // Binds movement to just the four cardinal directions
            if (!omniMode) {
                double moveInputX = 0;
                double moveInputY = 0;
                if (Math.abs(gamepad1.left_stick_x) > 0.6) {
                    moveInputX = Math.signum(gamepad1.left_stick_x) * SpeedCoefficients.getStrafeSpeed();
                    moveInputY = 0;
                } else if (Math.abs(gamepad1.left_stick_y) > 0.6) {
                    moveInputX = 0;
                    moveInputY = Math.signum(gamepad1.left_stick_y) * SpeedCoefficients.getForwardSpeed();
                } else {
                    moveInputX = gamepad1.left_stick_x;
                    moveInputY = gamepad1.left_stick_y;
                }
                driveX = moveInputX;
                driveY = moveInputY;
            }

            // Drive robot centric
            drive.driveRobotCentric(-driveX, driveY, -gamepad1.right_stick_x * SpeedCoefficients.getTurnSpeed());

            // Toggle sound w/ 'y' and circle
            if (gamepad1.circle && gamepad1.y && !soundToggled) {
                SoundPlayer.getInstance().stopPlayingAll();
                SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, heheha);
                soundToggled = true;
            }
            if (!gamepad1.circle && !gamepad1.y) soundToggled = false;



            //////////////////////////
            // Arm and Claw Control //
            //////////////////////////


            telemetry.addData("arm raised: ", inDep.getIsWristRaised());
            // Toggle claw with the 'y' button
            if ((gamepad1.y || gamepad2.y) && !clawToggled) {
                if (gamepad2.y) {
                    gamepad1.rumble(500);
                }
                if (inDep.getIsOpen()) {
                    inDep.close();
                } else {
                    inDep.open();
                }
                clawToggled = true;

            }
            if (!gamepad1.y && !gamepad2.y) clawToggled = false;

            // Toggle wrist with a condition
            boolean condition = inDep.getArmPosition() > 100 && (inDep.getArmPosition()-prevArmEncoder) >= 0; // first derivative
            if (condition) {
                inDep.lowerWrist();
            }
            if (!condition) {
                inDep.raiseWrist();
            }

            telemetry.addData("arm pos: ", inDep.getArmPosition());

            // Control arm power with triggers
            double totalChange = (gamepad1.right_trigger - gamepad1.left_trigger) * SpeedCoefficients.getArmSpeed();
            inDep.rawPower(totalChange);

            // Resets arm
            if (gamepad1.a || gamepad2.a) {

                inDep.resetArmEncoder();
                gamepad1.rumble(500);
                gamepad2.rumble(500);
            }


            telemetry.update();
        }
    }
}
