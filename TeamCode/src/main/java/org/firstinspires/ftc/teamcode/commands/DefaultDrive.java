package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import java.util.function.DoubleSupplier;

public class DefaultDrive extends CommandBase {
    private final DriveSubsystem drive;
    private final DoubleSupplier strafe;
    private final DoubleSupplier speed;
    private final DoubleSupplier rotation;

    public DefaultDrive(DriveSubsystem subsystem, DoubleSupplier strafe, DoubleSupplier speed, DoubleSupplier rotation) {
        drive = subsystem;
        this.strafe = strafe;
        this.speed = speed;
        this.rotation = rotation;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        drive.driveRobotCentric(strafe.getAsDouble(), speed.getAsDouble(), rotation.getAsDouble());
    }

}