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

@TeleOp(name = "mexican teleop")
public class MexicanTeleOp extends LinearOpMode {
    private final int TICKS_PER_SECOND = 20;
    private final long MILLISECOND_TOLERANCE = 5;

    @Override
    public void runOpMode() throws InterruptedException {
        // init robot
        HardwareRobot robot = new HardwareRobot(hardwareMap);
        SpeedCoefficients.setMode(SpeedCoefficients.MoveMode.MODE_FAST);
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

        SoundPlayer sfx = SoundPlayer.getInstance();
        sfx.stopPlayingAll();
        int clashStartup = hardwareMap.appContext.getResources().getIdentifier("clashroyaleintro", "raw", hardwareMap.appContext.getPackageName());
        int spanish = hardwareMap.appContext.getResources().getIdentifier("spanish", "raw", hardwareMap.appContext.getPackageName());
        int bongofeet = hardwareMap.appContext.getResources().getIdentifier("bongofeet", "raw", hardwareMap.appContext.getPackageName());
        int vineboom = hardwareMap.appContext.getResources().getIdentifier("vineboom", "raw", hardwareMap.appContext.getPackageName());

        sfx.startPlaying(hardwareMap.appContext, clashStartup);

        waitForStart();

        sfx.stopPlayingAll();


        boolean clawToggled = false, soundToggled = false;
        long TICK = -1, MOD = 0;
        // main TeleOp loop
        while (opModeIsActive()) {

            ////////////////////
            // Timing Control //
            ////////////////////

            Thread.sleep((long) 1000/TICKS_PER_SECOND - MILLISECOND_TOLERANCE); // 20 TPS
            TICK = TICK+1;





            //////////////////////////
            // Basic Motion Control //
            //////////////////////////

            // Speed control
            if (gamepad1.dpad_up) {
                for (int i = 0; i < 3; i++) gamepad1.rumble(166);
                SpeedCoefficients.setMode(SpeedCoefficients.MoveMode.MODE_FAST);
            } else if (gamepad1.dpad_down) {
                gamepad1.rumble(500);
                SpeedCoefficients.setMode(SpeedCoefficients.MoveMode.MODE_SLOW);
            }

            // Binds movement to just the four cardinal directions
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
            drive.driveRobotCentric(-moveInputX, moveInputY, -gamepad1.right_stick_x * SpeedCoefficients.getTurnSpeed()); // Drive Robot Centric





            ///////////////////
            // Sound Control //
            ///////////////////

            // looping spanish
            if (TICK % ((long) (120 * TICKS_PER_SECOND)) == 0)
                sfx.startPlaying(hardwareMap.appContext, spanish);

            // movement bongo feet
            if (Math.abs(moveInputX) >= 0.1 || Math.abs(moveInputY) >= 0.1) {
                if (!soundToggled) {
                    MOD = TICK % ((long) (0.9 * TICKS_PER_SECOND));
                    soundToggled = true;
                }
                if (TICK % ((long) (0.9 * TICKS_PER_SECOND)) == MOD)
                    sfx.startPlaying(hardwareMap.appContext, bongofeet);
            }
            if (Math.abs(moveInputX) < 0.1 && Math.abs(moveInputY) < 0.1) soundToggled = false;

            // claw vine boom
            if (gamepad1.y && !clawToggled)
                sfx.startPlaying(hardwareMap.appContext, vineboom);





            //////////////////////////
            // Arm and Claw Control //
            //////////////////////////

            // Toggle claw with the 'y' button
            if (gamepad1.y && !clawToggled) {
                if (inDep.getIsOpen()) {
                    inDep.close();
                } else {
                    inDep.open();
                }
                clawToggled = true;
            }
            if (!gamepad1.y) clawToggled = false;

            // Control arm power with triggers
            double totalChange = (gamepad1.right_trigger - gamepad1.left_trigger) * SpeedCoefficients.getArmSpeed();
            inDep.rawPower(totalChange);
        }
    }
}