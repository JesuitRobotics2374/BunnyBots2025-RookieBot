// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import frc.robot.Constants;


public class ShooterSubsystem extends SubsystemBase {
   //change later when more info is available 
  // motor type for flywheels is kraken 
  private TalonFX control;

  public ShooterSubsystem() {

    this.control = new TalonFX(37, "FastFD"); // change id when necessary
    
  }

  double targetSpeed = Constants.TARGET_SHOOTER_SPEED;
  double maxRPM = Constants.SHOOTER_MAX_RPM;
  double maxRPS = maxRPM / 60;

  public boolean checkSpeedShooter() {
    double motorRPS = control.getVelocity().getValueAsDouble();
    double motorSpeed = motorRPS / maxRPS;
    return motorSpeed == targetSpeed;

    // use this for shuffleboard

  }

  public void setSpeedShooter(double speed) {
    control.set(speed);
  }

  public boolean isShooterReady(boolean isCarrotLoaded) { // fix when indexer is made

    // indexer and shooter need to communicate about if 5 carrots are ready to be
    // shot

    if (checkSpeedShooter() /* && insert communication here */ ) {
      return true;
    } else {
      return false;
    }

  }

  public void shootCarrots(double rpmValue) {
      
      if (isShooterReady(true)) {
        
        setSpeedShooter(targetSpeed);

        // wait however long it takes for flywheel to reach targetSpeed
        // communicate with indexer

        //command here but i lwk forgot how to do that


      }

      else {
        System.out.println("SHOOTER NOT READY!");
      }
      
    // fix later
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
