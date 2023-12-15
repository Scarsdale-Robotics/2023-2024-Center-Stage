package org.firstinspires.ftc.teamcode.opmodes.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareRobot;
import org.firstinspires.ftc.teamcode.SpeedCoefficients;
import org.firstinspires.ftc.teamcode.subsystems.InDepSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.CVSubsystem;

@TeleOp(name = "televerything (telemetry everything)")
public class Televerything extends LinearOpMode {
    private final boolean isRedTeam = false;

    @Override
    public void runOpMode() {

        HardwareRobot robot = new HardwareRobot(hardwareMap);
        SpeedCoefficients.setMode(SpeedCoefficients.MoveMode.MODE_FAST);
        InDepSubsystem inDep = new InDepSubsystem(
                robot.arm1,
                robot.arm2,
                robot.rightClaw,
                robot.leftClaw,
                robot.wrist,
                robot.elbow,
                this,
                telemetry
        );
        DriveSubsystem drive = new DriveSubsystem(
                robot.leftFront,
                robot.rightFront,
                robot.leftBack,
                robot.rightBack,
                robot.imu,
                inDep,
                this
        );
        CVSubsystem cv = new CVSubsystem(
                robot.camera,
                robot.cameraName,
                drive, telemetry,
                isRedTeam,
                this
        );

        waitForStart();

        boolean omniToggle = false, omniMode = false, clawToggle = false, speedIsFast = false;
        double moveInputX,moveInputY,turnInput,DISTANCE_BEFORE_BACKBOARD = 45,cvDist;  // TEMP
        SpeedCoefficients.setMode(SpeedCoefficients.MoveMode.MODE_SLOW);
        while (opModeIsActive()) {

            ////////////////////
            // MOTION CONTROL //
            ////////////////////
            if (gamepad1.dpad_up) {
                SpeedCoefficients.setMode(SpeedCoefficients.MoveMode.MODE_FAST);
                speedIsFast = true;
            }
            if (gamepad1.dpad_down) {
                SpeedCoefficients.setMode(SpeedCoefficients.MoveMode.MODE_SLOW);
                speedIsFast = false;
            }

            //Toggle Omni Mode
            if (gamepad1.square && !omniToggle) {
                omniToggle = true;
                omniMode = !omniMode;
            }
            if (!gamepad1.square) omniToggle = false;

            // Drive Robot
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
            turnInput = gamepad1.right_stick_x;
            drive.driveRobotCentric(-moveInputX,moveInputY,-turnInput * SpeedCoefficients.getTurnSpeed());






            /////////////////
            // ARM CONTROL //
            /////////////////

            // Claw Toggling
            if (gamepad1.y && !clawToggle) {
                if (inDep.getIsLeftClawOpen())
                    inDep.close();
                else {
                    inDep.open();
                    // Set Fast Mode
                    SpeedCoefficients.setMode(SpeedCoefficients.MoveMode.MODE_FAST);
                }
                clawToggle = true;
            }
            if (!gamepad1.y) clawToggle = false;

            // Flexible Arm Control
            inDep.rawPower((gamepad1.left_trigger - gamepad1.right_trigger) * SpeedCoefficients.getArmSpeed(), gamepad1.a);

            // Emergency Reset Arm
            if (gamepad2.a) {
                inDep.resetArmEncoder();
                gamepad1.rumble(500);
                gamepad2.rumble(500);
            }






            /////////////////////
            // COMPUTER VISION //
            /////////////////////

            cvDist = cv.getAprilTagDistance(isRedTeam ? new Integer[] {4, 5, 6} : new Integer[] {1, 2, 3});

            // Crash Prevention
            if (!gamepad2.x && !gamepad1.x && cvDist < DISTANCE_BEFORE_BACKBOARD && !inDep.getIsLeftClawOpen()) {
                SpeedCoefficients.setMode(SpeedCoefficients.MoveMode.MODE_SLOW);
            } else if (gamepad1.x || gamepad2.x) {
                gamepad1.rumble(500);
                gamepad2.rumble(500);
            }





            ///////////////
            // TELEMETRY //
            ///////////////


            telemetry.addData("speedIsFast: ", speedIsFast);
            telemetry.addData("omniMode:", omniMode);
            telemetry.addData("moveInputX: ", moveInputX);
            telemetry.addData("moveInputY: ", moveInputY);
            telemetry.addData("turnInput: ", turnInput);
            telemetry.addData("wrist.getPosition: ", robot.wrist.getPosition());
            telemetry.addData("wrist.getPosition: ", robot.wrist.getPosition());
            telemetry.addData("leftClaw.getPosition: ", robot.leftClaw.getPosition());
            telemetry.addData("rightClaw.getPosition: ", robot.rightClaw.getPosition());
            telemetry.addData("arm.getCurrentPosition:", robot.arm1.motor.getCurrentPosition());
            telemetry.addData("arm.getPower: ", robot.arm1.motor.getPower());
            telemetry.addData("cvDist:", cvDist);
            telemetry.addData("rotOff: ",cv.getAprilTagRotationalOffset(isRedTeam ? 5 : 2));


            telemetry.addData("⠀⢸⠂⠀⠀⠀⠘⣧⠀⠀⣟⠛⠲⢤⡀⠀⠀⣰⠏⠀⠀⠀⠀⠀⢹⡀", 0);
            telemetry.addData("⠀⡿⠀⠀⠀⠀⠀⠈⢷⡀⢻⡀⠀⠀⠙⢦⣰⠏⠀⠀⠀⠀⠀⠀⢸⠀", 0);
            telemetry.addData("⠀⡇⠀⠀⠀⠀⠀⠀⢀⣻⠞⠛⠀⠀⠀⠀⠻⠀⠀⠀⠀⠀⠀⠀⢸⠀", 0);
            telemetry.addData("⠀⡇⠀⠀⠀⠀⠀⠀⠛⠓⠒⠓⠓⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢸⠀", 0);
            telemetry.addData("⠀⡇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣸⠀", 0);
            telemetry.addData("⠀⢿⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣀⣀⣀⣀⠀⠀⢀⡟⠀", 0);
            telemetry.addData("⠀⠘⣇⠀⠘⣿⠋⢹⠛⣿⡇⠀⠀⠀⠀⣿⣿⡇⠀⢳⠉⠀⣠⡾⠁⠀", 0);
            telemetry.addData("⣦⣤⣽⣆⢀⡇⠀⢸⡇⣾⡇⠀⠀⠀⠀⣿⣿⡷⠀⢸⡇⠐⠛⠛⣿⠀", 0);
            telemetry.addData("⠹⣦⠀⠀⠸⡇⠀⠸⣿⡿⠁⢀⡀⠀⠀⠿⠿⠃⠀⢸⠇⠀⢀⡾⠁⠀", 0);
            telemetry.addData("⠀⠈⡿⢠⢶⣡⡄⠀⠀⠀⠀⠉⠁⠀⠀⠀⠀⠀⣴⣧⠆⠀⢻⡄⠀⠀", 0);
            telemetry.addData("⠀⢸⠃⠀⠘⠉⠀⠀⠀⠠⣄⡴⠲⠶⠴⠃⠀⠀⠀⠉⡀⠀⠀⢻⡄⠀", 0);
            telemetry.addData("⠀⠘⠒⠒⠻⢦⣄⡀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣀⣤⠞⠛⠒⠛⠋⠁⠀", 0);
            telemetry.addData("⠀⠀⠀⠀⠀⠀⠸⣟⠓⠒⠂⠀⠀⠀⠀⠀⠈⢷⡀⠀⠀⠀⠀⠀⠀⠀", 0);
            telemetry.addData("⠀⠀⠀⠀⠀⠀⠀⠙⣦⠀⠀⠀⠀⠀⠀⠀⠀⠈⢷⠀⠀⠀⠀⠀⠀⠀", 0);
            telemetry.addData("⠀⠀⠀⠀⠀⠀⠀⣼⣃⡀⠀⠀⠀⠀⠀⠀⠀⠀⠘⣆⠀⠀⠀⠀⠀⠀", 0);
            telemetry.addData("⠀⠀⠀⠀⠀⠀⠀⠉⣹⠃⠀⠀⠀⠀⠀⠀⠀⠀⠀⢻⠀⠀⠀⠀⠀⠀", 0);
            telemetry.addData("⠀⠀⠀⠀⠀⠀⠀⠀⡿⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢸⡆⠀⠀⠀⠀⠀", 0);


            telemetry.update();
        }
    }
}
