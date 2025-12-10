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
import frc.robot.utils.Devices;

import com.ctre.phoenix6.hardware.core.CoreCANrange;

/**
 * to do:
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
  private int numOfCarrots = 0;
  
    private TalonFX entranceControl;
    private TalonFX exitControl;
  
    private CoreCANrange intakeSensor;
    private CoreCANrange exitSensor;
  
    private boolean detectionState = false;
    private boolean detectionStateOut;
  
    private double entranceSpeed;
    private double exitSpeed;

    
  
    public IndexerSubsystem() {
  
      this.entranceControl = Devices.INDEXER_ENTERANCE_MOTOR; // change id when necessary
      this.exitControl = Devices.INDEXER_EXIT_MOTOR; // change id when necessary
      this.intakeSensor = Devices.INDEXER_ENTERANCE_CANRANGE;
      this.exitSensor = Devices.INDEXER_EXIT_CANRANGE;
  
      entranceSpeed = Constants.CONVEYER_ENTRANCE_SPEED;
      exitSpeed = Constants.CONVEYER_EXIT_SPEED; //FIX LATER PLEASE

      isIndexFull = numOfCarrots >= 5;
  
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
    return isIndexFull;
  } // true = four carrots in indexer {}

  // /**
  // * Loads carrot into shooter.
  // *
  // * @param speed The speed of the motor when loading.
  // */
  // public void loadCarrot(int speed) {
  // control.set(0.2); // goal of this func is to create room in conveyor belt for
  // more carrots (move conveyor for short amount of time then turn off)
  // for (int i = 0; i<10000; i++ /*edit value later depending on how long we want
  // this to go */){
  // continue;
  // }
  // control.set(0);
  // }

  public void stop() {
    entranceControl.set(0);
    exitControl.set(0);
  }

  private void setBeltSpeed() {
    entranceControl.set(entranceSpeed);
    exitControl.set(exitSpeed);
  }

  private void setBeltSpeedBack() {
    entranceControl.set(-entranceSpeed);
    exitControl.set(-exitSpeed);
  }



  // public Command Advance() {
  //   setBeltSpeed();
  //       return null;
  // }

  public Command Purge() {
    return new InstantCommand(() -> setBeltSpeedBack(), this);
  }

  public Command StopBelt() {
    return new InstantCommand(() -> stop(), this);
  }

  public Command Advance() {
    return new FunctionalCommand(
      //init
      () -> {numOfCarrots = 0;
             stop();},
      //execute
      () -> {setBeltSpeed();
             updateDetectionStateIn();
            },
      //interrupt
      interrupted -> {stop();},
      //isFinished
      () -> isIndexFull,
      //requirements
      this
    );
  }

  // public Command Retreat() {
  //   setBeltSpeed();
  //       return null;
  // }

  // public Command Halt() {
  //   stop();
  //       return null;
  // }

  /**
   * Tells us if the carrot is loaded.
   * 
   * @return true = carrot is loaded.
   */ 
  // do not delete this
  public void updateDetectionStateIn() {
    boolean current = intakeSensor.getIsDetected().getValue();

    // Count only on rising edge (false -> true)
    if (!detectionState && current) {
        numOfCarrots++;
        
    }

    detectionState = current;
  }

  public void updateDetectionStateOut() {
    boolean current = exitSensor.getIsDetected().getValue();

    if (!detectionStateOut && current) {
        numOfCarrots--;
    }
    detectionStateOut = current;
  }

  public void updateAll() {

    updateDetectionStateIn();
    updateDetectionStateOut();
    // add more here if necessary (updating other vars)

  }

  public void advance() {

    exitControl.set(exitSpeed); // just a placeholder for now
    // tihs will run the conveyor belt for a longer time than usual to make sure all
    // carrots are shot, then we can go back to getting carrot!

  }

  @Override
  public void periodic() {
    updateAll();
  }

}
