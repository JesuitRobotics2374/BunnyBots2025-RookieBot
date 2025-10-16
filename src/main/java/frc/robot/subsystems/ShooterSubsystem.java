// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;

public class ShooterSubsystem extends SubsystemBase {
  private final targetRPM = 12; //change later when more info is available 
  // motor type for flywheels is kraken 
  private TalonFX control;

  public ShooterSubsystem() {

    this.control = new TalonFX(37, "FastFD"); // change id when necessary


    public boolean checkRpmShooter() {
      int motorRPS = control.getRotorVelocity().getValue();
      return motorRPS * 60 == targetRPM;
      // use this for shuffleboard 

    }

    public void setRpmShooter(int rpmValue) {
      control.set(rpmValue);
    }

    public boolean isShooterReady(boolean isCarrotLoaded) {

      // indexer and shooter need to communicate about if 5 carrots are ready to be shot

      if (checkRpmShooter()  /* && insert communication here*/ ) {
        return true;
      }
      else {
        return false;
      }


    }

    public void shootCarrots(int rpmValue) {
      
      setRpmShooter(targetRPM);
      // wait however long it takes for flywheel to reach targetRPM
      

    


    // fix later

    } 

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
