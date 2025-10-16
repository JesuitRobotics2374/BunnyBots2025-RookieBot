// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

  private TalonFX deployMotor;
  private TalonFX intakeMotor;

  private boolean intakeDeployed; // true = down
  private boolean intaking; // true = intaking
  private boolean isLoaded; // true = loaded

  /** Initializer - runs when class is created. */
  public IntakeSubsystem() {
    deployMotor = new TalonFX(5);
    intakeMotor = new TalonFX(6);

    deployMotor.setNeutralMode(NeutralModeValue.Brake);
    intakeMotor.setNeutralMode(NeutralModeValue.Coast);
    
    intaking = false;
  }

  /**
   * Deploy the intake (put the intake down).
   * @param deployed Tells if the intake is deployed. true = down.
   */
  public void setIntakeDeployed(boolean deployed) {
    
  }

  /**
   * Stops intaking.
   */
  public void stopIntake() {
    intaking = false;
    intakeMotor.stopMotor();
  }

  /**
   * Purge the intake so it gets rid of carrots.
   * @param speed The speed that the intake will go at.
   */
  public void purge(double speed) {
    intaking = false;
    intakeMotor.set(-speed);
  }

  /**
   * Intake the carrot by spinning the motor.
   * @param speed The speed that the motor will spin at.
   */
  public void intakeCarrot(double speed) {
    intaking = true;
    intakeMotor.set(speed);
  }

  /**
   * tells if intake is down or up.
   * @return true = down.
   */
  public boolean getIntakeDeployed() {
    return intakeDeployed;
  }

  /**
   * tells if intake is intaking. 
   * @return true = intaking.
   */
  public boolean getIntaking() {
    return intaking;
  }

  /**
   * tells if intake is loaded.
   * @return true = loaded.
   */
  public boolean getIsLoaded() {
    return isLoaded;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run.
  }
}
