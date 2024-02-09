package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;

public class EndgameSubsystem extends SubsystemBase {

    public static boolean droneReleased = false;
    private CRServo drone;

    public EndgameSubsystem(CRServo drone) {
        this.drone = drone;
        droneReleased = false;
        holdDrone();
    }

    public void setPower(double power) {
        drone.setPower(power);
    }

    public double getPower() {
        return drone.getPower();
    }

    public void holdDrone() {
        drone.setPower(0);
    }

    public void releaseDrone() {
        drone.setPower(1);
        droneReleased = true;
    }

}
