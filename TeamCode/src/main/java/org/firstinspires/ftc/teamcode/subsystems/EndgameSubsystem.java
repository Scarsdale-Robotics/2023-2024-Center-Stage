package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

public class EndgameSubsystem extends SubsystemBase {

    private Servo drone;

    public EndgameSubsystem(Servo drone) {
        this.drone = drone;

        drone.setPosition(0);
    }

    public void setPosition(double servoPos) {
        drone.setPosition(servoPos);
    }

    public void releaseDrone() {
        drone.setPosition(0);
    }
}
