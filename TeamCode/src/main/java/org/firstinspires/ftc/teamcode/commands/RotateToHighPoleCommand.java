//package org.firstinspires.ftc.teamcode.commands;
//import com.arcrobotics.ftclib.command.CommandBase;
//
//import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
//import com.arcrobotics.ftclib.hardware.RevIMU;
//
//
//
//public class RotateToHighPoleCommand extends CommandBase{
//    private final DriveSubsystem drive;
//    private final RevIMU imu;
//
//    public RotateToHighPoleCommand(DriveSubsystem drive){
//        this.drive = drive;
//        this.imu = drive.imu;
//        addRequirements(drive);
//    }
//
//    @Override
//    public void execute(){
//        drive.rotate(0.3);
//    }
//
//    @Override
//    public boolean isFinished(){
//        double startHeading = imu.getHeading();
//        return imu.getHeading() - startHeading >= 135;
//    }
//
//}
