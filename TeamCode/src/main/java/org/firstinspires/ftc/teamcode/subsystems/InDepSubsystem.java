package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.SpeedCoefficients;

public class InDepSubsystem extends SubsystemBase {
    private LinearOpMode opMode;
    private Motor arm;
    private Servo claw;
    private Servo wrist;
    private Level level;
    public enum Level {
        GROUND(50),
        BACKBOARD(250); //temp motor encoder values
        int target;

        Level(int target) {
            this.target = target;
        }
    }

    private boolean isRaised;
    private boolean isOpen;

    public InDepSubsystem(Motor arm, Servo claw, Servo wrist, LinearOpMode opMode) {
        // initialize objects
        this.arm = arm;
        this.claw = claw;
        this.wrist = wrist;
        this.opMode = opMode;

        // initialize vars
        level = Level.GROUND;
        isRaised = false;
        isOpen = false;

        // reset the arm and claw states
        lower();
        close();
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
    public boolean getIsRaised() {
        return isRaised;
    }

    /**
     * Raises the arm and flattens the wrist.
     */
    public void raise() {
        level = Level.BACKBOARD; // set current level to raised
        arm.setTargetPosition(level.target);

        wrist.setPosition(0.05); // set wrist position to flattened

        arm.set(SpeedCoefficients.getArmSpeed());
        while (opMode.opModeIsActive() && !arm.atTargetPosition()) ; // wait until arm is at target position

        // complete action
        arm.stopMotor();
        isRaised = true;
    }

    /**
     * Lowers the arm and angles the wrist.
     */
    public void lower() {
        level = Level.GROUND; // set current level to lowered
        arm.setTargetPosition(level.target);

        wrist.setPosition(0.15); // set wrist position to angled

        arm.set(SpeedCoefficients.getArmSpeed());
        while (opMode.opModeIsActive() && !arm.atTargetPosition()) ; // wait until arm is at target position

        // complete action
        arm.stopMotor();
        isRaised = false;
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
