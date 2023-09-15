package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.HardwareRobot;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

// Set the name for the OpMode that will appear on the driver station
@TeleOp(name = "Example TeleOp")
// Create class and basically include methods for opmode
public class ExTeleOp extends LinearOpMode {
    private double moveSpeed = 0.75;
    private double turnSpeed = 0.6;

    @Override
    // The "Main" code will go in here
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
        GamepadEx driverOp1 = new GamepadEx(gamepad1);

        waitForStart();

        boolean fieldCentric = false;
        while (opModeIsActive()) {
            if (gamepad1.a) {
                fieldCentric = !fieldCentric;
            }

            // Deal with movement inputs
            double driveX = driverOp1.getLeftX();
            double driveY = driverOp1.getLeftY();
            // If the positive difference in X/Y input is severe, lock on the biggest input direction
            if (Math.abs(driveX) - Math.abs(driveY) > 0.6) {
                // Lots more X input, lock on X-axis movement
                driveX = Math.signum(driveX);
                driveY = 0;
            } else if (Math.abs(driveY) - Math.abs(driveX) > 0.6) {
                // Lots more Y input, lock on Y-axis movement
                driveX = 0;
                driveY = Math.signum(driveY);
            }

            // Reset imu
            if (gamepad2.x && gamepad2.y) {
                drive.resetIMU();
                gamepad1.rumble(500);  // rumble brrrr
                gamepad2.rumble(500);
            }

            // Apply movement
            drive.drive(driveX, driveY, driverOp1.getRightX() * turnSpeed, fieldCentric);
        }
    }
/*
    private AprilVision initAprilTag() {
        // figure out initapriltag https://github.com/FIRST-Tech-Challenge/FtcRobotController/blob/master/FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/ConceptAprilTag.java
        AprilTagProcessor aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();
        // work in progresssss
    }
*/

    private class AprilVision {
        private AprilTagProcessor aprilTag;
        private VisionPortal visionPortal;
        private AprilVision(AprilTagProcessor aprilTag, VisionPortal visionPortal) {
            this.aprilTag = aprilTag;
            this.visionPortal = visionPortal;
        }
    }
}