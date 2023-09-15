//package org.firstinspires.ftc.teamcode.opmodes;
//
//import com.arcrobotics.ftclib.command.CommandOpMode;
//import com.arcrobotics.ftclib.command.InstantCommand;
//import com.arcrobotics.ftclib.command.ParallelCommandGroup;
//import com.arcrobotics.ftclib.command.SequentialCommandGroup;
//import com.arcrobotics.ftclib.gamepad.GamepadEx;
//import com.arcrobotics.ftclib.gamepad.GamepadKeys;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.teamcode.commands.DefaultDrive;
//import org.firstinspires.ftc.teamcode.commands.DriveBackCommand;
//import org.firstinspires.ftc.teamcode.commands.DriveForwardCommand;
//import org.firstinspires.ftc.teamcode.commands.LiftLevelDownCommand;
//import org.firstinspires.ftc.teamcode.commands.LiftLevelUpCommand;
//import org.firstinspires.ftc.teamcode.commands.MaintainLiftCommand;
//import org.firstinspires.ftc.teamcode.commands.MoveLiftCommand;
//import org.firstinspires.ftc.teamcode.HardwareRobot;
//import org.firstinspires.ftc.teamcode.commands.RotateToHighPoleCommand;
//import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;
//
//@TeleOp(name = "Sample Command TeleOp")
//public class CommandTeleOp extends CommandOpMode {
//    private HardwareRobot robot;
//    private DefaultDrive driveCommand;
//    private DriveSubsystem drive;
//    private ClawSubsystem claw;
//    private GamepadEx driverOp;
//    private LiftSubsystem lift;
//
//    @Override
//    public void initialize() {
//        robot = new HardwareRobot(hardwareMap);
//        driverOp = new GamepadEx(gamepad1);
//
//        drive = new DriveSubsystem(
//                robot.leftFront,
//                robot.rightFront,
//                robot.leftBack,
//                robot.rightBack,
//                robot.imu,
//                this
//        );
//
//        lift = new LiftSubsystem(robot.lift);
//
//        driveCommand = new DefaultDrive(drive, driverOp::getLeftX, driverOp::getLeftY, driverOp::getRightX);
//
//        SequentialCommandGroup highPolePosition = new SequentialCommandGroup(
//                new ParallelCommandGroup(
//                        new DriveBackCommand(drive),
//                        new MoveLiftCommand(lift, LiftSubsystem.Level.HIGH)
//                ),
//                new RotateToHighPoleCommand(drive),
//                new DriveForwardCommand(drive)
//        );
//
//        driverOp.getGamepadButton(GamepadKeys.Button.Y).toggleWhenPressed(
//                new InstantCommand(claw::open, claw),
//                new InstantCommand(claw::close, claw)
//        );
//
//        driverOp.getGamepadButton(GamepadKeys.Button.A).whenPressed(new LiftLevelUpCommand(lift));
//        driverOp.getGamepadButton(GamepadKeys.Button.B).whenPressed(new LiftLevelDownCommand(lift));
//        driverOp.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(highPolePosition);
//
//        register(drive);
//        drive.setDefaultCommand(driveCommand);
//        lift.setDefaultCommand(new MaintainLiftCommand(lift));
//    }
//}