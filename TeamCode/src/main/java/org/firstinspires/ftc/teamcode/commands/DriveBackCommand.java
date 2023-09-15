package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import com.arcrobotics.ftclib.hardware.motors.Motor;


public class DriveBackCommand extends CommandBase {
    private final DriveSubsystem drive;
    private final Motor rightBack;

    public DriveBackCommand(DriveSubsystem drive) {
        this.drive = drive;
        this.rightBack = drive.rightBack;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        drive.drive(-0.3);
    }

    @Override
    public boolean isFinished() {
        double startEncoder = rightBack.getCurrentPosition();
        return Math.abs(rightBack.getCurrentPosition() - startEncoder) >= 1000;
    }
}
