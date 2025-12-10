package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision.VisionSubsystem;

public class ForwardUntilTag extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final VisionSubsystem visionSubsystem;

    private boolean done;

    public ForwardUntilTag(CommandSwerveDrivetrain driveSubsystem, VisionSubsystem visionSubsystem) {
        this.drivetrain = driveSubsystem;
        this.visionSubsystem = visionSubsystem;

        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        done = false;
    }

    @Override
    public void execute() {
        drivetrain.setControl(
                new SwerveRequest.RobotCentric().withVelocityY(-Constants.AUTO_STATIC_MOVE_SPEED)); // pray this works, reason is we start sideways so that vision starts aligned on yaw

        if (visionSubsystem.getAllVisibleTagIDs().size() > 0) {
            done = true;
        }
    }

    @Override
    public boolean isFinished() {
        return done;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Move Back complete!");
        drivetrain.setControl(new SwerveRequest.SwerveDriveBrake());
    }

}