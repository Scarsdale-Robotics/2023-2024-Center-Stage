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
        GROUND(0),
        BACKBOARD(70); //temp motor encoder values
        int target;

        Level(int target) {
            this.target = target;
        }
    }

    private boolean isWristRaised;
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
        isWristRaised = true;
        isArmRaised = false;
        isOpen = false;

        // reset the arm and claw states
        raiseWrist();
        lowerArm();
        close();
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
    public boolean getIsOpen() {
        return isOpen;
    }

    /**
     * @return true if the arm is raised, otherwise false if it is lowered.
     */
    public boolean getIsWristRaised() {
        return isWristRaised;
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
     * Raises the wrist.
     */
    public void raiseWrist() {
        wrist.setPosition(0); // set wrist position to angled
        isWristRaised = true;
    }

    /**
     * Lowers the wrist.
     */
    public void lowerWrist() {
        wrist.setPosition(0.15); // set wrist position to flattened
        isWristRaised = false;
    }

    /**
     * Raises the arm.
     */
    public void raiseArm() {
        level = Level.BACKBOARD; // set current level to raised
        arm.setTargetPosition(level.target);

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
        level = Level.GROUND; // set current level to lowered
        arm.setTargetPosition(level.target);

        while (opMode.opModeIsActive() && !arm.atTargetPosition()) {
            arm.setTargetPosition(level.target);
            arm.set(SpeedCoefficients.getArmSpeed());
        }; // wait until arm is at target position

        // complete action
        arm.stopMotor();
        isArmRaised = false;
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
}
