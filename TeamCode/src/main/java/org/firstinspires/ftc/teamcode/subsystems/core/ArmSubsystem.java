package org.firstinspires.ftc.teamcode.subsystems.core;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

public class ArmSubsystem extends SubsystemBase {
    public final Servo servo;
    public Level level;
    public enum Level {
        GROUND(0.05), BACKBOARD(0.5); //temp values

        public final double target;

        Level(double target) {
            this.target = target;
        }
    }
    public ArmSubsystem(Servo servo) {
        this.servo = servo;
        setLevel(Level.GROUND);
    }

    public void setLevel(Level level) {
        this.level = level;
        servo.setPosition(level.target);
    }

    public Level getLevel() {
        return level;
    }
}
