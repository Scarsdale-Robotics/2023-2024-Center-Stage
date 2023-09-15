package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

public class LiftLevelDownCommand extends CommandBase {
    private final LiftSubsystem lift;

    public LiftLevelDownCommand(LiftSubsystem subsystem) {
        lift = subsystem;
        addRequirements(lift);
    }

    @Override
    public void initialize(){
        lift.nextLevelDown();
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
