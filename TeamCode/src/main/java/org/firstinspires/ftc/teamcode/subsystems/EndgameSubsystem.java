package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.SpeedCoefficients;

public class EndgameSubsystem extends SubsystemBase {

//    private final Motor lift1;
//    private final Motor lift2;
    public enum Level {
        GROUND(0),
        HANG(999999999);
        int target;
        Level(int target) {
            this.target = target;
        }
    }

//    public EndGameSubsystem(Motor lift1, Motor lift2) {
//        //Initializes the subsystem
//        this.lift1 = lift1;
//        this.lift2 = lift2;
//    }
}
