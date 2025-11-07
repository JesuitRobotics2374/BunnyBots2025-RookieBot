package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.hardware.Pigeon2;
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
    private double currentYaw;

    boolean finishedOverride;

    Pigeon2 pigeon = new Pigeon2(0);

    public TurnCommand(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;

        yaw_offset = Math.PI/2;
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
        pigeon.reset();

    }

    @Override
    public void execute() {
        drivetrain.setControl(driveRequest.withRotationalRate(dtheta));

        currentYaw = pigeon.getYaw().getValueAsDouble();

        dtheta = yawController.calculate(currentYaw, yaw_offset);
        dtheta = Math.max(-MAX_ANGULAR_SPEED, Math.min(dtheta * THETA_SPEED_MODIFIER, MAX_ANGULAR_SPEED));
        dtheta = yawRateLimiter.calculate(dtheta);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(currentYaw - yaw_offset) < 0.05; // ~3 degrees tolerance
        // set end command
    }

    @Override
    public void end(boolean interrupted) {
        // drivetrain.setControl(new SwerveRequest.SwerveDriveBrake());
        // stop the robot
    }
}

/*
    Some issues to consider:
    Relative vs. absolute turn – Your current code always tries to go to an absolute 90°, not “turn 90° from wherever you are.” If your robot starts at a different heading, it won’t turn the intended amount.

    First loop dtheta bug – You calculate dtheta after sending it to the drivetrain. On the first iteration, the robot won’t turn.

    Wraparound issues – If your current yaw is near ±π, the isFinished() check can fail or trigger too early.

    MIN_ANGULAR_COMMAND not used – Very small commands might not move the robot if your motors have a deadzone.
 */