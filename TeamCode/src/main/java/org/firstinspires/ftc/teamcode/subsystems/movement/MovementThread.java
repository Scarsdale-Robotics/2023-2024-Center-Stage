package org.firstinspires.ftc.teamcode.subsystems.movement;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.CVSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.InDepSubsystem;
import org.firstinspires.ftc.teamcode.util.SpeedCoefficients;

public class MovementThread implements Runnable {
    public static volatile double K_FORWARD = 64.04;
    public static volatile double K_STRAFE = 72.948;
    public static volatile double K_ARM = 11.111;

    private static final double POWER_DRIVE = SpeedCoefficients.getAutonomousDriveSpeed();
    private static final double POWER_TURN = SpeedCoefficients.getAutonomousTurnSpeed();
    private static final double POWER_ARM = SpeedCoefficients.getAutonomousArmSpeed();

    private final ElapsedTime runtime;
    private final Movement movement;
    private static volatile DriveSubsystem drive;
    private static volatile InDepSubsystem inDep;
    private static volatile CVSubsystem cv;
    private static volatile LinearOpMode opMode;

    public MovementThread(Movement movement) {
        this.movement = movement;
        runtime = new ElapsedTime();
        runtime.reset();

    }

    public static void initSubsystems(DriveSubsystem drive, InDepSubsystem inDep, CVSubsystem cv, LinearOpMode opMode) {
        MovementThread.drive = drive;
        MovementThread.inDep = inDep;
        MovementThread.opMode = opMode;
        MovementThread.cv = cv;
    }

    @Override
    public void run() {
        Movement.MovementType type = movement.MOVEMENT_TYPE;

        // DRIVE CASES
        if (type.isDriveType) {
            double  a = movement.INCHES_FORWARD * type.SGN_forward,
                    b = movement.INCHES_STRAFE * type.SGN_strafe,
                    c = Math.hypot(a,b),
                    theta = Math.atan2(a,b),

                    u = Math.hypot(MovementThread.K_STRAFE * Math.cos(theta), MovementThread.K_FORWARD * Math.sin(theta)),
                    L = Math.abs(u*c*Math.cos(theta)-u*c*Math.sin(theta))/Math.sqrt(2),
                    R = Math.abs(u*c*Math.cos(theta)+u*c*Math.sin(theta))/Math.sqrt(2);

            drive.driveByAngularEncoder(POWER_DRIVE, L, R, theta);
        }

        // TURN CASES
        if (type.isTurnType) {
            drive.turnByIMU(POWER_TURN, movement.DEGREES_TURN * type.SGN_turn);
        }

        // ARM CASES
        if (type.isArmType) {
            inDep.raiseByEncoder(
                POWER_ARM,
                movement.DEGREES_ELEVATION * MovementThread.K_ARM * type.SGN_elevation
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

        // ELBOW CASES
        if (type == Movement.MovementType.REST_ELBOW) {
            inDep.rest();
        }
        if (type == Movement.MovementType.FLIP_ELBOW) {
            inDep.flip();
        }

        // CV CASES
        if (type.isCVType) {
            if (type == Movement.MovementType.WHITE_PXL_ALIGN)
                cv.moveToWhitePixel();
            if (type == Movement.MovementType.APRIL_TAG_ALIGN)
                cv.moveToWhitePixel();
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
