package org.firstinspires.ftc.teamcode.opmodes.calibration;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareRobot;
import org.firstinspires.ftc.teamcode.subsystems.RobotSystem;
import org.firstinspires.ftc.teamcode.util.SpeedCoefficients;
import org.firstinspires.ftc.teamcode.subsystems.InDepSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.CVSubsystem;

@TeleOp(name = "televerything (telemetry everything)")
public class Televerything extends LinearOpMode {
    private final boolean isRedTeam = false;
    private DriveSubsystem drive;
    private InDepSubsystem inDep;
    private CVSubsystem cvBack;

    @Override
    public void runOpMode() {
        RobotSystem robot = new RobotSystem(hardwareMap, false, this, telemetry);
        robot.getInDep().close();
        drive = robot.getDrive();
        inDep = robot.getInDep();

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
            inDep.rawPower((gamepad1.left_trigger - gamepad1.right_trigger) * SpeedCoefficients.getArmSpeed());

            // Emergency Reset Arm
            if (gamepad2.a) {
                inDep.resetArmEncoder();
                gamepad1.rumble(500);
                gamepad2.rumble(500);
            }






            /////////////////////
            // COMPUTER VISION //
            /////////////////////

            cvDist = cvBack.getAprilTagDistance(isRedTeam ? new Integer[] {4, 5, 6} : new Integer[] {1, 2, 3});

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
            telemetry.addData("wrist.getPosition: ", inDep.getWristPosition());
            telemetry.addData("elbow.getPosition: ", inDep.getElbowPosition());
            telemetry.addData("leftClaw.getPosition: ", inDep.getLeftClawPosition());
            telemetry.addData("rightClaw.getPosition: ", inDep.getRightClawPosition());
            telemetry.addData("arm.getCurrentPosition:", inDep.getArmPosition());
            telemetry.addData("arm.getPower: ", inDep.getArmVelocity());
            telemetry.addData("cvDist:", cvDist);
            telemetry.addData("rotOff: ",cvBack.getAprilTagRotationalOffset(isRedTeam ? 5 : 2));

/* why just why

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
*/

            telemetry.update();
        }
    }
}
