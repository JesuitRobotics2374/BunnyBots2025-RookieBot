// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Falcon motor 

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants; // hey theres a constants file in the shooter branch this references that
import com.ctre.phoenix6.hardware.core.CoreCANrange;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.CANBus;      // only if specifying CAN bus name





/** todo:
 * Make control work
 * Set the conveyor belt move (public void shootcarrots)
 * 
 * actually do things with the carrot value (letsgo i got that to work :)
 * 
 *  
 * 
*/

public class IndexerSubsystem extends SubsystemBase {
  /** Creates a new IndexerSubsystem. */

  private boolean isIndexFull;
  private int numOfCarrots;
  private TalonFX entranceControl;
  private TalonFX exitControl;  
  private CoreCANrange intakeSensor;
  private CoreCANrange shooterSensor;
  private boolean detectionState;
  private boolean detectionStateOut;


  public IndexerSubsystem() {
    numOfCarrots = 0;
    this.entranceControl = new TalonFX(16, "rio"); // change id when necessary
    this.exitControl = new TalonFX(5, "rio"); // change id when necessary
    //this.intakeSensor = new CoreCANrange(67, "FastFD"); // change id when necessary 
    //this.shooterSensor = new CoreCANrange(67, "FastFD"); // change id when necessary 
    

  }

  /**
   * Shows us the number of carrots in the indexer.
   * 
   * @return Integer of the number of carrots.
   */
  public int getNumOfCarrots() {
    return numOfCarrots;
  }

  /**
   * Tells us when the indexer is full.
   * 
   * @return true = indexer is full.
   */
  public boolean getIsIndexFull() {

    if (getNumOfCarrots() == 4) {
      isIndexFull = true;
    }
    else {
      isIndexFull = false;
    }

    return isIndexFull;
  } // true = four carrots in indexer {}

  // /**
  //  * Loads carrot into shooter.
  //  * 
  //  * @param speed The speed of the motor when loading.
  //  */
  // public void loadCarrot(int speed) {
  // control.set(0.2); // goal of this func is to create room in conveyor belt for more carrots (move conveyor for short amount of time then turn off)  
  // for (int i = 0; i<10000; i++ /*edit value later depending on how long we want this to go */){
  //     continue;
  // }
  // control.set(0);
  // }

  public void stop() {
    entranceControl.set(0);
    exitControl.set(0);
  }

  public void setBeltSpeed(double speed) {
      entranceControl.set(speed);
      exitControl.set(speed);
  }

  public Command moveBelt() {
    return new InstantCommand(() -> this.setBeltSpeed(0.1), this);
  }

  public Command stopBelt() {
    return new InstantCommand(() -> this.stop(), this);
  }

  /**
   * Tells us if the carrot is loaded.
   * 
   * @return true = carrot is loaded.
   */
 
  public void updateDetectionStateIn(){

    if (detectionState != intakeSensor.getIsDetected().getValue()){

      if (detectionState == false){

        numOfCarrots++;

      } 
  
    }

    detectionState = intakeSensor.getIsDetected().getValue();

  } 

  public void updateDetectionStateOut(){

    if (detectionStateOut != shooterSensor.getIsDetected().getValue()){

      if (detectionStateOut == true){

        numOfCarrots--;

      } 
  
    }

    detectionStateOut = shooterSensor.getIsDetected().getValue();

  }


  public void updateAll(){

    updateDetectionStateIn();
    updateDetectionStateOut();
    // add more here if necessary (updating other vars)

  }

  public void shootCarrots() {
    

    exitControl.set(Constants.CONVEYOR_TARGET_SPEED); //just a placeholder for now
    // tihs will run the conveyor belt for a longer time than usual to make sure all carrots are shot, then we can go back to getting carrot!

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateAll();
    

  }
}
