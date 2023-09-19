package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.util.Range;
import com.arcrobotics.ftclib.controller.PIDFController;


public class LiftSubsystem extends SubsystemBase {
    private final Motor m;
    private Level level;

    public enum Level {
        PICKUP(30),
        STACK_2(248),
        STACK_3(465),
        STACK_4(683),
        STACK_5(900),
        GROUND(350),
        LOW(1800),
        MEDIUM(3050),
        HIGH(4175);


        public final int target;

        Level(int target) {
            this.target = target;
        }

        public Level nextAbove() {
            if (this == PICKUP) return GROUND;
            else if (this == GROUND) return LOW;
            else if (this == LOW) return MEDIUM;
            else return HIGH; // For MEDIUM and HIGH
        }

        public Level nextBelow() {
            if (this == PICKUP) return PICKUP;
            else if (this == GROUND) return PICKUP;
            else if (this == LOW) return GROUND;
            else if (this == MEDIUM) return LOW;
            else return MEDIUM;
        }
    }

    public LiftSubsystem(Motor liftMotor) {
        m = liftMotor;
        level = Level.PICKUP;
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

    public void nextLevelUp() {
        setLevel(level.nextAbove());
    }

    public void nextLevelDown() {
        setLevel(level.nextBelow());
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
        m.setTargetPosition(Level.PICKUP.target);
    }
}