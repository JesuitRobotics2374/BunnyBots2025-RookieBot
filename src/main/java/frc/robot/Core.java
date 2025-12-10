// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.path.PathConstraints;
// import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import frc.robot.seafinder2.PathfinderSubsystem;
// import frc.robot.seafinder2.interfaces.PanelSubsystem;
import frc.robot.utils.Target;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Vision.VisionSubsystem;
//import frc.robot.seafinder2.SF2Constants;
import frc.robot.commands.ExactAlign;
// import frc.robot.seafinder2.commands.ScoreCommand;
// import frc.robot.seafinder2.commands.TestCommand;
// import frc.robot.seafinder2.commands.limbControl.EjectCommand;
// import frc.robot.seafinder2.commands.limbControl.ElevatorCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class Core {

    public double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * Constants.MAX_SPEED;

    public double MaxSpeedTurbo = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * Constants.MAX_SPEED_TURBO;

    public boolean isTurbo = false;

    public double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond) * Constants.MAX_ANGULAR_RATE;

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.02) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final CommandXboxController driveController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
    public final IndexerSubsystem m_IndexerSubsystem = new IndexerSubsystem();
    public final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();
    public final VisionSubsystem m_visionSubsystem = new VisionSubsystem();

    public SequentialCommandGroup autoCommandGroup;

    Pose3d llp;
    Pose3d llp2;

    private Target target;

    public AprilTagFieldLayout atf;

    public Core() {

        // target = new Target(this);
        // target.setLocation(new Target.Location(Landmark.REEF_BACK, Side.RIGHT));
        // target.setHeight(Target.Height.BRANCH_L4); // This is a structural requirement, but we don't use it here.

        registerAutoCommands();
        configureBindings();
        configureShuffleBoard();

        atf = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

        ShuffleboardTab tab = Shuffleboard.getTab("Test");

        if (llp != null) {

            tab.addDouble("EE LL X", () -> {
                return llp.getX();
            });
            tab.addDouble("EE LL Y", () -> {
                return llp.getY();
            });
            tab.addDouble("EE LL Yaw", () -> {
                return llp.getRotation().getZ();
            });

        }
    }


    public void registerAutoCommands() {
    }

    public void configureShuffleBoard() {

        ShuffleboardTab tab = Shuffleboard.getTab("Test");

        tab.addDouble("Robot Y", () -> drivetrain.getRobotY());
        // .withWidget("Number Bar");
        tab.addDouble("Robot X", () -> drivetrain.getRobotX());

        tab.addBoolean("FAST MODE", () -> {
            return isTurbo;
        });

    }

    private void configureBindings() {

        // STICK MOVEMENT
        drivetrain.setDefaultCommand(
                drivetrain.applyRequest(() -> drive
                        // .withVelocityX(-driveController.getLeftY() * Constants.MAX_SPEED *
                        // getAxisMovementScale())
                        // .withVelocityY(-driveController.getLeftX() * Constants.MAX_SPEED *
                        // getAxisMovementScale())
                        .withVelocityX(-driveController.getLeftY() * (isTurbo ? MaxSpeedTurbo : MaxSpeed)
                                * getAxisMovementScale() //* elevatorSlowSpeed()
                                )
                        .withVelocityY(-driveController.getLeftX() * (isTurbo ? MaxSpeedTurbo : MaxSpeed)
                                * getAxisMovementScale() //* elevatorSlowSpeed()
                                )
                        .withRotationalRate(-driveController.getRightX() * MaxAngularRate * getAxisMovementScale())));

        driveController.back().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric())); // RESET POSE


        // operatorController.a().onTrue(m_ShooterSubsystem.shootCarrots());
        // operatorController.b().onTrue(m_ShooterSubsystem.stopShooter());
        // operatorController.x().onTrue(m_IndexerSubsystem.moveBelt());
        // operatorController.y().onTrue(m_IndexerSubsystem.stopBelt());

        // operatorController.a().whileTrue(m_IntakeSubsystem.Intake()).onFalse(m_IntakeSubsystem.Stop());
        // operatorController.x().whileTrue(m_IntakeSubsystem.Purge()).onFalse(m_IntakeSubsystem.Stop());
        // operatorController.y().onTrue(m_IndexerSubsystem.Advance());
        // operatorController.b().onTrue(m_ShooterSubsystem.shootCarrots());

        operatorController.a().onTrue(m_IntakeSubsystem.Intake());
        operatorController.povDown().onTrue(m_IntakeSubsystem.Stop());
        operatorController.povLeft().onTrue(m_IntakeSubsystem.Purge()); //test out later

        operatorController.b().onTrue(new ParallelCommandGroup(m_IndexerSubsystem.Advance(), m_ShooterSubsystem.shootCarrots()));
        operatorController.povRight().onTrue(new ParallelCommandGroup(m_IndexerSubsystem.StopBelt(), m_ShooterSubsystem.stopCarrots()));        
        operatorController.x().onTrue(m_ShooterSubsystem.stopCarrots());
        
        operatorController.y().onTrue(new ParallelCommandGroup(m_IntakeSubsystem.Intake(), m_IndexerSubsystem.Advance()));
        operatorController.povUp().onTrue(m_IndexerSubsystem.Purge()); //We need a command to turn off the shooter
            
         // operatorController.y().onTrue(m_ShooterSubsystem.stopCarrots());
        //operatorController.x().whileTrue(m_ShooterSubsystem.shootCarrots()).onFalse(m_IndexerSubsystem.stop());  
        //driveController.b().onTrue(new InstantCommand(() -> target.cycleLocationRight()));
        //driveController.a().onTrue(new InstantCommand(() -> target.cycleLocationLeft()));

    }

    public void forwardAlign() {
    }

    public CommandSwerveDrivetrain getDrivetrain() {
        return drivetrain;
    }

    public double getAxisMovementScale() {
        return (1 - (driveController.getRightTriggerAxis() * 0.85));
    }

    int clock = 0;

    public void corePeriodic() {
    }
}