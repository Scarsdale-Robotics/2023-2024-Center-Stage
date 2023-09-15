package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

public class MaintainLiftCommand extends CommandBase {
    private final LiftSubsystem lift;

    public MaintainLiftCommand(LiftSubsystem subsystem) {
        lift = subsystem;
    }

    @Override
    public void execute() {
        lift.runLift();
    }
}
