// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Devices;

public class IntakeSubsystem extends SubsystemBase {

  private SparkMax intakeMotor;
  private IndexerSubsystem m_IndexerSubsystem = new IndexerSubsystem();

  private boolean intaking; // true = intaking

  /** Initializer - runs when class is created. */
  public IntakeSubsystem() {
    intakeMotor = Devices.INTAKE_MOTOR;
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(IdleMode.kCoast);

    intakeMotor.configure(config, null, null);
    intaking = false;
  }

  /**
   * Stops intaking.
   */
  private void stopIntake() {
    intaking = false;
    intakeMotor.stopMotor();
  }

  /**
   * Purge the intake so it gets rid of carrots.
   * @param speed The speed that the intake will go at.
   */
  private void purge(double speed) {
    intaking = false;
    intakeMotor.set(-speed);
  }

  /**
   * Intake the carrot by spinning the motor.
   * @param speed The speed that the motor will spin at.
   */
  private void intakeCarrot(double speed) {
    intaking = true;
    intakeMotor.set(speed);
  }

  /**
   * tells if intake is intaking. 
   * @return true = intaking.
   */
  public boolean getIntaking() {
    return intaking;
  }

  // public Command Intake() {
  //   return new InstantCommand(() -> intakeCarrot(0.15), this);
  // }

  public Command Intake() {
    return new FunctionalCommand(
      //init
      () -> {stopIntake();},
      //execute
      () -> {intakeCarrot(0.2);
             m_IndexerSubsystem.Advance();},
      //interrupt
      interrupted -> {stopIntake();},
      //isFinished
      () -> m_IndexerSubsystem.getIsIndexFull(),
      //requirements
      this, m_IndexerSubsystem
    );
  }

  public Command Purge() {
    return new InstantCommand(() -> purge(0.3), this);
  }

  public Command Stop() {
    return new InstantCommand(() -> stopIntake(), this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run.
  }
}
