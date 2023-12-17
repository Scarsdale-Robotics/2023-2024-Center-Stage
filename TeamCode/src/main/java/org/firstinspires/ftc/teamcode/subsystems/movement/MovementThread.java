package org.firstinspires.ftc.teamcode.subsystems.movement;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.CVSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.InDepSubsystem;
import org.firstinspires.ftc.teamcode.util.SpeedCoefficients;

public class MovementThread implements Runnable {
    private static final double TICKS_PER_INCH_FORWARD = 32.4;
    private static final double TICKS_PER_INCH_STRAFE = 62.5;
    private static final double TICKS_PER_DEGREE_TURN = 10.0;
    private static final double TICKS_PER_DEGREE_ARM = 35.0;

    private static final double POWER_FORWARD = SpeedCoefficients.getAutonomousForwardSpeed();
    private static final double POWER_STRAFE = SpeedCoefficients.getAutonomousStrafeSpeed();
    private static final double POWER_TURN = SpeedCoefficients.getAutonomousTurnSpeed();
    private static final double POWER_ARM = SpeedCoefficients.getAutonomousArmSpeed();

    final private ElapsedTime runtime;
    final private Movement movement;
    final private DriveSubsystem drive;
    final private InDepSubsystem inDep;
    final private LinearOpMode opMode;
    final private CVSubsystem cvBack;

    public MovementThread(Movement movement, DriveSubsystem drive, InDepSubsystem inDep, CVSubsystem cvBack, LinearOpMode opMode) {
        this.movement = movement;
        this.drive = drive;
        this.inDep = inDep;
        this.cvBack = cvBack;
        this.opMode = opMode;
        runtime = new ElapsedTime();
        runtime.reset();
    }

    @Override
    public void run() {
        Movement.MovementType type = movement.MOVEMENT_TYPE;

        // GENERIC DRIVE CASES
        drive.driveByEncoder(
                POWER_STRAFE * type.K_strafe,
                POWER_FORWARD * type.K_forward,
                POWER_TURN * type.K_turn,
                Math.sqrt(
                        // Pythagorean Theorem for diagonal movement
                        Math.pow(movement.INCHES_STRAFE * TICKS_PER_INCH_STRAFE * Math.abs(type.K_strafe),2) +
                        Math.pow(movement.INCHES_FORWARD * TICKS_PER_INCH_FORWARD * Math.abs(type.K_forward),2)
                ) +
                movement.DEGREES_TURN * TICKS_PER_DEGREE_TURN * Math.abs(type.K_turn)
        );

        // ELEVATION CASES
        inDep.raiseByEncoder(
                POWER_ARM * type.K_elevation,
                movement.DEGREES_ELEVATION * TICKS_PER_DEGREE_ARM * Math.abs(type.K_elevation)
        );

        // DELAY CASE
        if (type == Movement.MovementType.DELAY)
            sleepFor(movement.WAIT);

        // CLAW CASES
        if (type == Movement.MovementType.OPEN_LEFT_CLAW)
            inDep.openLeft();
        if (type == Movement.MovementType.OPEN_RIGHT_CLAW)
            inDep.openRight();
        if (type == Movement.MovementType.CLOSE_RIGHT_CLAW)
            inDep.closeRight();
        if (type == Movement.MovementType.CLOSE_LEFT_CLAW)
            inDep.closeLeft();

        // CV CASES
        if (type == Movement.MovementType.WHITE_PXL_ALIGN)
            cvBack.moveToWhitePixel();
    }

    /**
     * Smart sleep with opMode running check.
     * @param ms Timeout in milliseconds.
     */
    private void sleepFor(long ms) {
        runtime.reset();
        while (opMode.opModeIsActive() && (runtime.milliseconds() < ms));
    }
}