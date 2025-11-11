package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision.VisionSubsystem;
import frc.robot.utils.Target.TagRelativePose;

public class AlignCommand extends SequentialCommandGroup {

    public AlignCommand(CommandSwerveDrivetrain drivetrain, VisionSubsystem vision, TagRelativePose tagPose) {
      addCommands(
          new ExactAlign(drivetrain, vision, tagPose),
          new TurnCommand(drivetrain)
      );
    }
  }
  