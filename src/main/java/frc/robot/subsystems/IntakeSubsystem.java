// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

  private SparkMax intakeMotor;

  private boolean intaking; // true = intaking

  /** Initializer - runs when class is created. */
  public IntakeSubsystem() {
    intakeMotor = new SparkMax(6, MotorType.kBrushless);
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(IdleMode.kCoast);

    intakeMotor.configure(config, null, null);
    
    intaking = false;
  }

  /**
   * Deploy the intake (put the intake down).
   * @param deployed Tells if the intake is deployed. true = down.
   */
  // public void setIntakeDeployed(boolean deployed) {
  //   MotionMagicVoltage m_request;
  //   if (deployed) {
  //     m_request = new MotionMagicVoltage(Constants.INTAKE_DEPLOYED_POSITION);
  //     intakeDeployed = true;
  //   } else {
  //     m_request = new MotionMagicVoltage(Constants.INTAKE_RETRACTED_POSITION);
  //     intakeDeployed = false;
  //   }
  // }

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
   * tells if intake is intaking. 
   * @return true = intaking.
   */
  public boolean getIntaking() {
    return intaking;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run.
  }
}
