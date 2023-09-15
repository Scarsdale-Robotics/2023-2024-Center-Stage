package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

public class LiftLevelUpCommand extends CommandBase {
    private final LiftSubsystem lift;

    public LiftLevelUpCommand(LiftSubsystem subsystem) {
        lift = subsystem;
        addRequirements(lift);
    }

    @Override
    public void initialize(){
        lift.nextLevelUp();
    }

    @Override
    public void execute(){
        lift.runLift();
    }

    @Override
    public boolean isFinished() {
        return lift.atTarget();
    }
}
