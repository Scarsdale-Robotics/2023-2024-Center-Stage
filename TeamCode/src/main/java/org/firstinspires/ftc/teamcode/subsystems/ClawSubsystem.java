package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

public class ClawSubsystem extends SubsystemBase {
    public final Servo servo;

    public ClawSubsystem(Servo servo) {
        this.servo = servo;
    }

    public void open() {
        servo.setPosition(0.175);
    }

    public void close() {
        servo.setPosition(0.0);
    }
}
