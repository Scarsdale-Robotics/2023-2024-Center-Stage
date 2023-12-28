package org.firstinspires.ftc.teamcode.subsystems.movement;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.CVSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.InDepSubsystem;
import org.firstinspires.ftc.teamcode.util.SpeedCoefficients;

public class MovementThread implements Runnable {
    private static double K_FORWARD = 32.4;
    private static double K_STRAFE = 62.5;
    private static double K_ARM = 35.0;

    private static final double POWER_DRIVE = SpeedCoefficients.getAutonomousDriveSpeed();
    private static final double POWER_TURN = SpeedCoefficients.getAutonomousTurnSpeed();
    private static final double POWER_ARM = SpeedCoefficients.getAutonomousArmSpeed();

    private final ElapsedTime runtime;
    private final Movement movement;
    private static volatile DriveSubsystem drive;
    private static volatile InDepSubsystem inDep;
    private static volatile CVSubsystem cvFront;
    private static volatile CVSubsystem cvBack;
    private static volatile LinearOpMode opMode;

    public MovementThread(Movement movement) {
        this.movement = movement;
        runtime = new ElapsedTime();
        runtime.reset();

    }

    public static void initSubsystems(DriveSubsystem drive, InDepSubsystem inDep, CVSubsystem cvFront, CVSubsystem cvBack, LinearOpMode opMode) {
        MovementThread.drive = drive;
        MovementThread.inDep = inDep;
        MovementThread.opMode = opMode;
        MovementThread.cvFront = cvFront;
        MovementThread.cvBack = cvBack;
    }

    @Override
    public void run() {
        Movement.MovementType type = movement.MOVEMENT_TYPE;

        // DRIVE CASES
        if (type.isDriveType) {
            double  a = movement.INCHES_FORWARD * type.SGN_forward,
                    b = movement.INCHES_STRAFE * type.SGN_strafe,
                    c = Math.hypot(a,b),
                    theta = Math.atan2(b,a),

                    u = Math.hypot(K_STRAFE * Math.cos(theta), K_FORWARD * Math.sin(theta)),
                    L = Math.abs(u*c*Math.cos(theta)-u*c*Math.sin(theta))/Math.sqrt(2),
                    R = Math.abs(u*c*Math.cos(theta)+u*c*Math.sin(theta))/Math.sqrt(2);

            drive.driveByAngularEncoder(POWER_DRIVE, L, R, theta);
        }

        // TURN CASES
        if (type.isTurnType) {
            drive.turnByIMU(POWER_TURN, movement.DEGREES_TURN);
        }

        // ARM CASES
        if (type.isArmType) {
            inDep.raiseByEncoder(
                POWER_ARM * type.SGN_elevation,
                movement.DEGREES_ELEVATION * K_ARM
            );
        }

        // DELAY CASE
        if (type == Movement.MovementType.DELAY)
            sleepFor(movement.WAIT);

        // CLAW CASES
        if (type.isClawType) {
            if (type == Movement.MovementType.OPEN_LEFT_CLAW)
                inDep.openLeft();
            if (type == Movement.MovementType.OPEN_RIGHT_CLAW)
                inDep.openRight();
            if (type == Movement.MovementType.CLOSE_RIGHT_CLAW)
                inDep.closeRight();
            if (type == Movement.MovementType.CLOSE_LEFT_CLAW)
                inDep.closeLeft();
        }

        // CV CASES
        if (type.isCVType) {
            if (type == Movement.MovementType.WHITE_PXL_ALIGN)
                cvFront.moveToWhitePixel();
            if (type == Movement.MovementType.APRIL_TAG_ALIGN)
                cvBack.moveToWhitePixel();
        }
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
