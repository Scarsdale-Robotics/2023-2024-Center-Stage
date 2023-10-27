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
        GROUND(50), BACKBOARD(500); //temp motor encoder values

        int target;

        Level(int target) {
            this.target = target;
        }
    }

    public InDepSubsystem(Motor arm, Servo claw, Servo wrist, LinearOpMode opMode) {
        this.arm = arm;
        this.claw = claw;
        this.wrist = wrist;
        this.opMode = opMode;
        level = Level.GROUND;
    }

    public Level getLevel() {
        return level;
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

        arm.stopMotor();
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

        arm.stopMotor();
    }

    /**
     * Opens the claw.
     */
    private void open() {
        claw.setPosition(0.175);
    }

    /**
     * Closes the claw.
     */
    private void close() {
        claw.setPosition(0.0);
    }
}
