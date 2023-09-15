package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

public class MoveLiftCommand extends CommandBase{
    private final LiftSubsystem lift;
    private final LiftSubsystem.Level level;

    public MoveLiftCommand(LiftSubsystem subsystem, LiftSubsystem.Level level) {
        lift = subsystem;
        this.level = level;
        addRequirements(lift);
    }

    @Override
    public void initialize(){
        lift.setLevel(level);
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
