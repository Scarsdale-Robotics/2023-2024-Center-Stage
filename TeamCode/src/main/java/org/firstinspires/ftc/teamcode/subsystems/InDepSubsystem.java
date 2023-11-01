package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.SpeedCoefficients;

public class InDepSubsystem extends SubsystemBase {
    private LinearOpMode opMode;
    private Telemetry telemetry;
    private Motor arm;
    private Servo claw;
    private Servo wrist;
    private Level level;
    public enum Level {
        GROUND(0, 0.0),
        BACKBOARD1(70,0.05),
        BACKBOARD2(150,0.15); //temp motor encoder values

        int target;
        double wristTarget;

        Level(int target, double wristTarget) {
            this.target = target;
            this.wristTarget = wristTarget;
        }
    }

    private boolean isArmRaised;
    private boolean isOpen;

    public InDepSubsystem(Motor arm, Servo claw, Servo wrist, LinearOpMode opMode, Telemetry telemetry) {
        // initialize objects
        this.arm = arm;
        this.claw = claw;
        this.wrist = wrist;
        this.opMode = opMode;
        this.telemetry = telemetry;

        // initialize vars
        level = Level.GROUND;
        isArmRaised = false;
        isOpen = false;

        // reset the arm and claw states
        close();
        setLevel(level.target, level.wristTarget);
    }

    /**
     * @return the position of the arm motor, in ticks.
     */
    public int getArmPosition() {
        return arm.getCurrentPosition();
    }

    /**
     * @return true if the claw is open, otherwise false if it is closed.
     */

    //public double getWristPosition(){return wrist.}
    public boolean getIsOpen() {
        return isOpen;
    }

    /**
     * @return true if the arm is raised, otherwise false if it is lowered.
     */
    public boolean getIsArmRaised() {
        return isArmRaised;
    }

    /**
     * sets the raw power of the arm.
     */
    public void rawPower(double power) {
        arm.motor.setPower(power);
    }

    /**
     * Raises the arm.
     */
    public void raiseArm() {
        switch (level) {
            case GROUND:
                level = level.BACKBOARD1; // set current level to raised
                break;
            case BACKBOARD1:
                level = level.BACKBOARD2; // set current level to raised
                break;
            case BACKBOARD2:
                //setLevel(Level.BACKBOARD3);
                break;
        }
        setLevel(level.target, level.wristTarget);

        while (opMode.opModeIsActive() && !arm.atTargetPosition()) {
            arm.setTargetPosition(level.target);
            arm.set(SpeedCoefficients.getArmSpeed());
        }; // wait until arm is at target position

        // complete action
        arm.stopMotor();
        isArmRaised = true;
    }

    /**
     * Lowers the arm.
     */
    public void lowerArm() {
        switch (level) {
            case BACKBOARD2:
                level = level.BACKBOARD1; // set current level to lowered
                break;
            case BACKBOARD1:
                level = level.GROUND; // set current level to lowered
                break;
            case GROUND:
                //setLevel(Level.BACKBOARD3);
                break;
        }
        setLevel(level.target, level.wristTarget);

        while (opMode.opModeIsActive() && !arm.atTargetPosition()) {
            arm.setTargetPosition(level.target);
            arm.set(SpeedCoefficients.getArmSpeed());
        }; // wait until arm is at target position

        // complete action
        arm.stopMotor();
        isArmRaised = false;
    }

    public void setLevel(int target, double wristTarget) {
        arm.setTargetPosition(target);
        wrist.setPosition(wristTarget);
    }

    public void resetArmEncoder() {
        arm.resetEncoder();
    }

    /**
     * Opens the claw.
     */
    public void open() {
        claw.setPosition(0.175);
        isOpen = true;
    }

    /**
     * Closes the claw.
     */
    public void close() {
        claw.setPosition(0.0);
        isOpen = false;
    }

    /**
     * @return the level of the arm.
     */
    public Level getLevel() {
        return level;
    }

    public void changeElevation(int ticks) {
        int newTarget = getArmPosition() + ticks;
        arm.setTargetPosition(newTarget);

        while (opMode.opModeIsActive() && !arm.atTargetPosition()) {
            arm.set(SpeedCoefficients.getArmSpeed());
        }

        arm.stopMotor();
    }
}


