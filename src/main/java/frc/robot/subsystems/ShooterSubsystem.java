// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import frc.robot.Constants;


/*
 * NOTICE: SOME INFORMATION REQUIRES INDEXER SUBSYSTEM
 */
public class ShooterSubsystem extends SubsystemBase {
   //change later when more info is available 
  // motor type for flywheels is kraken 
  private TalonFX bottomWheel;
  private TalonFX topWheel;

  public ShooterSubsystem() {

    this.bottomWheel = new TalonFX(19, "rio"); // change id when necessary
    this.topWheel = new TalonFX(51, "rio"); //change id when necessary
    
  }

  double targetSpeed = Constants.TARGET_SHOOTER_SPEED;
  // double maxRPM = Constants.SHOOTER_MAX_RPM;
  // double maxRPS = maxRPM / 60;

  // public boolean checkSpeedShooter() { //STILL NEEDS TO BE FIXED 
  //   //might wanna add tolerance
  //   double motorRPS1 = bottomWheel.getVelocity().getValueAsDouble();
  //   double motorRPS2 = bottomWheel.getVelocity().getValueAsDouble();
  //   double motorSpeed1 = motorRPS1 / maxRPS;
  //   double motorSpeed2 = motorRPS2 / maxRPS;
  //   return motorSpeed1 == targetSpeed && motorSpeed2 == targetSpeed;

  //   // use this for shuffleboard

  // }

  /**
   * Moves both motors in opposite direction to shoot carrot.
   * @param speed
   */
  public void setSpeedShooter(double speed) {
    bottomWheel.set(speed);
    topWheel.set(-speed);
  }

  public void stop() {
    setSpeedShooter(0);
  }

  public boolean isShooterReady(boolean isCarrotLoaded) { // fix when indexer is made

    // indexer and shooter need to communicate about if 5 carrots are ready to be
    // shot

    // if (checkSpeedShooter() /* && insert communication here */ ) {
    //   return true;
    // } else {
    //   return false;
    // }

    if (isCarrotLoaded == true) {
      return true;
    } else {
      return false;
    }

  }

  public Command stopShooter() {
    return new InstantCommand(() -> this.stop(), this);
  }

  public Command shootCarrots() {
      
      //if (isShooterReady(true)) { //JUST FOR NOW MUST FIX

        return new InstantCommand(() -> this.setSpeedShooter(targetSpeed), this);

      //}

      // else {
      //   System.out.println("SHOOTER NOT READY!");
      //   return null;
      // }
      
    // fix later
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
