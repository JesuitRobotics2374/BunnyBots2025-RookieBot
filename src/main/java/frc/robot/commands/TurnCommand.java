package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

//50 ticks per second
// run for one second
// hint: use a clock

public class TurnCommand extends Command {
    private final SwerveRequest.RobotCentric driveRequest = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
   
    private CommandSwerveDrivetrain drivetrain;

    private final PIDController yawController;

    private final SlewRateLimiter yawRateLimiter = new SlewRateLimiter(100.0);

    private static final double THETA_SPEED_MODIFIER = 0.75;

    private static final double MAX_ANGULAR_SPEED = 0.5;
    private static final double MIN_ANGULAR_COMMAND = 0.25;

    private final double yaw_offset;
    private double dtheta;

    boolean finishedOverride;

    public TurnCommand(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        this.yaw_offset = Math.PI / 2;

        yawController = new PIDController(2.2, 0.1, 0.2);
        yawController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize() {

        finishedOverride = false;

        System.out.println("TURNCOMMAND STARTED");

        // Reset controllers and rate limiters
        yawController.reset();
        yawRateLimiter.reset(0);

    }

    public void execute() {
        dtheta = 0;

        // dtheta = yawController.calculate(avg_yaw, yaw_offset);
    }

    public boolean isFinished() {
        return true; //change in a bit
        // set end command
    }

    public void end(boolean interrupted) {
        // drivetrain.setControl(new SwerveRequest.SwerveDriveBrake());
        // stop the robot
    }
}