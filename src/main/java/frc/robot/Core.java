// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import javax.sound.sampled.SourceDataLine;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.AlignCommand;
import frc.robot.commands.ExactAlign;
import frc.robot.commands.TurnCommand;
import frc.robot.generated.SwerveeTunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.utils.Target;
import frc.robot.utils.Target.Landmark;
import frc.robot.utils.Target.Side;
import frc.robot.utils.Target.TagRelativePose;

public class Core {
    private double MaxSpeed = SwerveeTunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.5; // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    
    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driveController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = SwerveeTunerConstants.createDrivetrain();

    private Target target;
    public AprilTagFieldLayout atf;


    public Core() {
        //registerAutoCommands();
        // autoChooser = AutoBuilder.buildAutoChooser();
        configureBindings();
        //configureShuffleBoard();

        VisionSubsystem.initializeVisionSubsystem();


        // target = new Target(this);
        // target.setLocation(new Target.Location(Landmark.REEF_BACK, Side.RIGHT));
        // target.setHeight(Target.Height.BRANCH_L4); // This is a structural requirement, but we don't use it here.
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(driveController.getLeftY() * MaxSpeed * getAxisMovementScale()) // Drive forward with negative Y (forward)
                    .withVelocityY(driveController.getLeftX() * MaxSpeed * getAxisMovementScale()) // Drive left with negative X (left)
                    .withRotationalRate(driveController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        TagRelativePose testingTagRelativePose = new TagRelativePose(15, 0
        , 0, 0.0); // idk what units this is in - x is left
        // right & y is front back
        // currently working with oscillation
        driveController.a().onTrue(new AlignCommand(drivetrain, testingTagRelativePose));
    
        // driveController.x().onTrue(new SequentialCommandGroup(
        //     new ExactAlign(drivetrain, target.getTagRelativePose())
        //     //new ScoreCommand(target.getSetpoint(), elevatorSubsystem, manipulatorSubsystem)
        // ));


        driveController.back().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric())); // RESET POSE
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }

    public double getAxisMovementScale() {
        return (1 - (driveController.getRightTriggerAxis() * 0.75));
    }
}
