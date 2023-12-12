package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.SpeedCoefficients;
import org.firstinspires.ftc.teamcode.subsystems.movement.Movement;
import org.firstinspires.ftc.teamcode.subsystems.movement.MovementSequence;

import java.util.ArrayDeque;

public class DriveSubsystem extends SubsystemBase {
    private static final double TICKS_PER_INCH_FORWARD = 32.4;
    private static final double TICKS_PER_INCH_STRAFE = 62.5;
    private static final double TICKS_PER_DEGREE = 10.0;
    private static final double TICKS_PER_DEGREE_ARM = 35.0;
    private MecanumDrive controller;
    private IMU imu;
    private LinearOpMode opMode;
    private Motor rightBack;
    private InDepSubsystem inDep;

    public DriveSubsystem(Motor leftFront, Motor rightFront, Motor leftBack, Motor rightBack, IMU imu, InDepSubsystem inDep, LinearOpMode opMode) {
        this.rightBack = rightBack;
        controller = new MecanumDrive(
                leftFront,
                rightFront,
                leftBack,
                rightBack
        );
        this.imu = imu;
        this.opMode = opMode;
        this.inDep = inDep;
    }

    /**
     * Drives with directions based on robot pov.
     *
     * @param right     How much right the robot should strafe (negative values = strafe left).
     * @param forward   How much forward the robot should move (negative values = move backwards).
     * @param turn      How much the robot should turn.
     */
    public void driveRobotCentric(double right, double forward, double turn) {
        controller.driveRobotCentric(right, forward, turn);
    }

    /**
     * Drives with directions based on driver pov.
     *
     * @param right     How much right the robot should strafe (negative values = strafe left).
     * @param forward   How much forward the robot should move (negative values = move backwards).
     * @param turn      How much the robot should turn.
     */
    public void driveFieldCentric(double right, double forward, double turn) {
        controller.driveFieldCentric(right, forward, turn, getYaw());
    }

    /**
     * Use only for autonomous. Move a certain distance in ticks. Drive is robot centric.
     * @param rightSpeed      How fast the robot should strafe to the right (negative values = strafe left).
     * @param forwardSpeed  How fast the robot should move forward (negative values = strafe backwards).
     * @param turnSpeed      How fast the robot should turn (clockwise?).
     * @param ticks          How far the robot should move.
     */
    public void driveByEncoder(double rightSpeed, double forwardSpeed, double turnSpeed, double ticks) {
        double startEncoder = rightBack.getCurrentPosition();

        while (opMode.opModeIsActive() && Math.abs(rightBack.getCurrentPosition() - startEncoder) < ticks) {
            driveRobotCentric(rightSpeed, forwardSpeed, turnSpeed);
        }

        controller.stop();
    }

    /**
     * Use only for autonomous. Follow the passed MovementSequence. Drive is robot centric.
     * @param movementSequence      The MovementSequence to be followed.
     */
    public void followMovementSequence(MovementSequence movementSequence) throws InterruptedException {
        double  POWER_FORWARD = SpeedCoefficients.getAutonomousForwardSpeed(),
                POWER_STRAFE = SpeedCoefficients.getAutonomousStrafeSpeed(),
                POWER_TURN = SpeedCoefficients.getAutonomousTurnSpeed();
        ArrayDeque<Movement> movements = movementSequence.movements.clone();

        while (opMode.opModeIsActive() && !movements.isEmpty()) {
            Movement movement = movements.pollFirst();
            switch (movement.MOVEMENT_TYPE) {
                // forward
                case 0: driveByEncoder(0, POWER_FORWARD, 0, movement.INCHES_FORWARD * TICKS_PER_INCH_FORWARD);
                        break;
                // backward
                case 1: driveByEncoder(0, -POWER_FORWARD, 0, movement.INCHES_FORWARD * TICKS_PER_INCH_FORWARD);
                        break;
                // left
                case 2: driveByEncoder(-POWER_STRAFE, 0, 0, movement.INCHES_STRAFE * TICKS_PER_INCH_STRAFE);
                        break;
                // right
                case 3: driveByEncoder(POWER_STRAFE, 0, 0, movement.INCHES_STRAFE * TICKS_PER_INCH_STRAFE);
                        break;
                // turn left
                case 4: driveByEncoder(0, 0, -POWER_TURN, movement.DEGREES_TURN * TICKS_PER_DEGREE);
                        break;
                // turn right
                case 5: driveByEncoder(0, 0, POWER_TURN, movement.DEGREES_TURN * TICKS_PER_DEGREE);
                        break;
                // delay/no movement
                case 6: Thread.sleep(movement.WAIT);
                        break;
                // close claw
                case 7: inDep.close();
                        break;
                // open claw
                case 8: inDep.open();
                        break;
                // open claw
                case 9: inDep.changeElevation(movement.ELEVATION * TICKS_PER_DEGREE_ARM);
                        break;
                // invalid
                default: controller.stop();
                        break;
            }
        }

        controller.stop();
    }

    /**
     * @return the current position of the robot's wheels in ticks.
     */
    public int getWheelPosition() {
        return rightBack.getCurrentPosition();
    }

    public void resetIMU() {
        imu.resetYaw();
    }

    private double getYaw() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }
}