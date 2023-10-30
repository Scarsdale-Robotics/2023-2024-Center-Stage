package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.checkerframework.checker.units.qual.Speed;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.HardwareRobot;
import org.firstinspires.ftc.teamcode.SpeedCoefficients;
import org.firstinspires.ftc.teamcode.subsystems.InDepSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.CVSubsystem;

@TeleOp(name = "Drive TeleOp")
public class DriveTeleOp extends LinearOpMode {
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



        waitForStart();
        boolean isRedTeam = false;



        // main TeleOp loop
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


            //handle override and moving
            if (gamepad2.left_stick_y < -0.6 && override == false) {
                override = true;
            }

            if (cv.isRobotBeforeTape(isRedTeam)) { // detecting tape
                // turn off override
                if (!override)
                    moving = 0;
                else
                    moving = 1;
            } else { // not detecting tape
                override = false;
                moving = 1;
            }
            if (moving == 0) {
                //rumble
            }
            //moving = 0 while driverassist is active

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





            //////////////////////////
            // Arm and Claw Control //
            //////////////////////////


            telemetry.addData("arm raised: ", inDep.getIsWristRaised());
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
            if (gamepad2.a && gamepad2.b) {
                inDep.resetArmEncoder();
                gamepad1.rumble(500);
                gamepad2.rumble(500);
            }



            ///////////////////////
            // CV Motion Control //
            ///////////////////////

            // align with apriltag
            if (gamepad2.dpad_down == true && gamepad2.dpad_left == true && gamepad2.dpad_right == true && gamepad2.dpad_up == true) {
                if (isRedTeam) {
                    cv.alignParallelWithAprilTag(5);
                } else {
                    cv.alignParallelWithAprilTag(2);
                }
            }
            if (gamepad2.circle) { // picking up
                //override driver assist
                override = true;
                //ground position arm
                inDep.lowerArm();
                //set slow speed
                SpeedCoefficients.setMode(0);
                //align to april tag
                if (isRedTeam) {
                    cv.alignParallelWithAprilTag(5);
                } else {
                    cv.alignParallelWithAprilTag(2);
                }
            }
            if (gamepad2.square) {
                //override driver assist
                override = true;
                //set slow speed
                SpeedCoefficients.setMode(0);
                //align to april tag
                if (isRedTeam) {
                    cv.alignParallelWithAprilTag(5);
                } else {
                    cv.alignParallelWithAprilTag(2);
                }
            }

            prevArmEncoder = inDep.getArmPosition();
            telemetry.update();
        }
    }
}
