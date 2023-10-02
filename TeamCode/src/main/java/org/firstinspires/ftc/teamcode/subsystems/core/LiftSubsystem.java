package org.firstinspires.ftc.teamcode.subsystems.core;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.util.Range;
import com.arcrobotics.ftclib.controller.PIDFController;


public class LiftSubsystem extends SubsystemBase {
    private final Motor m;
    private Level level;

    public enum Level {
        GROUND(30), //TEMP
        BACKBOARD(500); // TEMP


        public final int target;

        Level(int target) {
            this.target = target;
        }
    }

    public LiftSubsystem(Motor liftMotor) {
        m = liftMotor;
        level = Level.GROUND;
    }

    public void runLift(double speed) {
        m.set(speed);
    }

    public void runLift() {
        m.set(1);
    }

    public void setLevel(Level level) {
        this.level = level;
        m.setTargetPosition(level.target);
    }

    public void setMode(Motor.RunMode runMode) {
        m.setRunMode(runMode);
    }

    public void setTargetToCurrent() {
        m.setTargetPosition(getPosition());
    }

    public int getPosition() {
        return m.getCurrentPosition();
    }

    public boolean atTarget() {
        return m.atTargetPosition();
    }

    public Level getLevel() {
        return level;
    }

    public void resetEncoder() {
        m.resetEncoder();
        m.setTargetPosition(Level.GROUND.target);
    }
}