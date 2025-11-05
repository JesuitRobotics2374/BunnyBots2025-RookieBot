package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utils.Target.TagRelativePose;

public class AlignCommand extends SequentialCommandGroup {

    public AlignCommand(CommandSwerveDrivetrain drivetrain, TagRelativePose tagPose) {
      addCommands(
          new ExactAlign(drivetrain, tagPose),
          new TurnCommand(drivetrain)
      );
    }
  }
  