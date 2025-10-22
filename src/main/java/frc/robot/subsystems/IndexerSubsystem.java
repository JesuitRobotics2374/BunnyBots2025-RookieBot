// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Falcon motor 

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants; // hey theres a constants file in the shooter branch this references that


/** todo:
 * Make control work
 * Set the conveyor belt move (public void shootcarrots)
 */

public class IndexerSubsystem extends SubsystemBase {
  /** Creates a new IndexerSubsystem. */

  private boolean isIndexFull;
  private int numOfCarrots;
  private TalonFX control; 

  public IndexerSubsystem() {
    numOfCarrots = 0;
    this.control = new TalonFX(35, "FastFD"); // change id when necessary

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

  /**
   * Loads carrot into shooter.
   * 
   * @param speed The speed of the motor when loading.
   */
  public void loadCarrot(int speed) {
  control.set(0.2); // goal of this func is to create room in conveyor belt for more carrots (move conveyor for short amount of time then turn off)  
  for (int i = 0; i<10000; i++ /*edit value later depending on how long we want this to go */){
      continue;
  }
  control.set(0);
  }

  /**
   * Tells us if the carrot is loaded.
   * 
   * @return true = carrot is loaded.
   */
 
  

  // name the one in indexer subsystem like conditionalShootCarrots or something
  // because that checks before shooting :)

  public void shootCarrots() {
    

    control.set(Constants.CONVEYOR_TARGET_SPEED); //just a placeholder for now
    // tihs will run the conveyor belt for a longer time than usual to make sure all carrots are shot, then we can go back to getting carrot!

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
