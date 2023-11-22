package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.SpeedCoefficients;

public class InDepSubsystem extends SubsystemBase {
    private final double CLAW_OPEN_POS = 0;
    private final double CLAW_CLOSED_POS = 0.175;
    private final double errorTolerance = 200;
    private final LinearOpMode opMode;
    private final Motor arm;
    private final Servo claw;
    private final Servo wrist;
    private boolean isClawOpen;
    public enum Level {
        GROUND(0, 0),
        BACKBOARD1(-1960,0.05),
        BACKBOARD2(-4200,0.125); //temp motor encoder values

        int target;
        double wristTarget;

        Level(int target, double wristTarget) {
            this.target = target;
            this.wristTarget = wristTarget;
        }
    }

    public InDepSubsystem(Motor arm, Servo claw, Servo wrist, LinearOpMode opMode, Telemetry telemetry) {
        // initialize objects
        this.arm = arm;
        this.claw = claw;
        this.wrist = wrist;
        this.opMode = opMode;

        // reset the arm and claw states
        close();
        setLevel(Level.GROUND);
    }

    /**
     * @return the position of the arm motor, in ticks.
     */
    public int getArmPosition() {
        return arm.motor.getCurrentPosition();
    }

    /**
     * @return true if the claw is open, otherwise false if it is closed.
     */

    //public double getWristPosition(){return wrist.}
    public boolean getIsOpen() {
        return isClawOpen;
    }


    /**
     * sets the raw power of the arm.
     */
    public void rawPower(double power) {
        int armPos = arm.motor.getCurrentPosition();

        // set bounds
//        if (armPos>=0 && power>0) { // more down is more positive
//            arm.motor.setPower(0);
//        } else if (armPos<=Level.BACKBOARD2.target && power<0) {
//            arm.motor.setPower(0);
//        } else {
//            arm.motor.setPower(power);
//        }

        arm.motor.setPower(power);

        // code moved into this method to avoid an edge case where curr pos moves slightly too much or fails to move enough
        // which would mess up curr pos ranges potentially making the wrist act unexpectedly
        // also moving here increases flexibility and avoids hard coding values without
        // having very long variable names


        if (armPos <= Level.BACKBOARD2.target) {
            wrist.setPosition(Level.BACKBOARD2.wristTarget);
        } else if (armPos <= Level.BACKBOARD1.target) {
            wrist.setPosition(Level.BACKBOARD1.wristTarget);
        } else {
            wrist.setPosition(Level.GROUND.wristTarget);
        }
    }

    /**
     * gets the level immediately above or immediately below the current position
     * @param getAbove whether to get the above level or below level
     * @return the nearby level specified (by getAbove)
     */
    private Level getNearbyLevel(boolean getAbove) {
        double armPos = arm.getCurrentPosition();
        if (armPos > Level.BACKBOARD2.target)
            return getAbove ? Level.BACKBOARD2 : Level.BACKBOARD1;
        else if (armPos > Level.BACKBOARD1.target)
            return getAbove ? Level.BACKBOARD2 : Level.GROUND;
        else
            return getAbove ? Level.BACKBOARD1 : Level.GROUND;
    }

    /**
     * Raises the arm.
     */
    public void raiseArm() {
        setLevel(getNearbyLevel(true));
    }

    /**
     * Lowers the arm.
     */
    public void lowerArm() {
        setLevel(getNearbyLevel(false));
    }

    public void setLevel(Level level) {
        arm.setTargetPosition(level.target);
        wrist.setPosition(level.wristTarget);
        arm.set(SpeedCoefficients.getArmSpeed());
    }


    public void resetArmEncoder() {
        arm.resetEncoder();
    }

    /**
     * Opens the claw.
     */
    public void open() {
        claw.setPosition(CLAW_OPEN_POS);
        isClawOpen = true;
    }

    /**
     * Closes the claw.
     */
    public void close() {
        claw.setPosition(CLAW_CLOSED_POS);
        isClawOpen = false;
    }
    public void changeElevation(int ticks) {
        int target = arm.motor.getCurrentPosition() - ticks;
        arm.setTargetPosition(target);
        arm.set(SpeedCoefficients.getArmSpeed());

        // wait until reached target within errorTolerance
        while (opMode.opModeIsActive() && !(Math.abs(target-arm.getCurrentPosition()) < errorTolerance) );

        // stop when reached
        arm.stopMotor();
        arm.motor.setPower(0);

    }

    public void resetArm() {
        int target = 0;
        arm.setTargetPosition(target);
        arm.set(SpeedCoefficients.getArmSpeed());

        // wait until reached target within errorTolerance
        while (opMode.opModeIsActive() && !(Math.abs(target-arm.getCurrentPosition()) < errorTolerance) );

        // stop when reached
        arm.stopMotor();
        arm.motor.setPower(0);

    }

    public void changeElevationDeg(double degrees) {
        // Conversion factor (ticks per degree)
        final double ticksPerDegree = 4200.0 / 120; // Replace 'maxDegrees' with the max degrees the arm can move

        // Convert degrees to ticks
        int ticks = (int) (degrees * ticksPerDegree);

        // Calculate target position in ticks
        int target = arm.motor.getCurrentPosition() - ticks;
        arm.setTargetPosition(target);
        arm.set(SpeedCoefficients.getArmSpeed());

        // Wait until the arm reaches the target within error tolerance
        while (!(Math.abs(target - arm.getCurrentPosition()) < errorTolerance));

        // Stop the motor once the target is reached
        arm.stopMotor();
        arm.motor.setPower(0);
    }
}


