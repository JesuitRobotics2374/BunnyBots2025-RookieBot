// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexerSubsystem extends SubsystemBase {
  /** Creates a new IndexerSubsystem. */

  private boolean isCarrotLoaded; 
  private boolean isIndexFull;
  private int numOfCarrots;

  public IndexerSubsystem() {
  numOfCarrots = 0;
  }

  /**
   * Shows us the number of carrots in the indexer.
   * @return Integer of the number of carrots.
   */
  public int getNumOfCarrots() {
    return numOfCarrots;
  }

  /**
   * Tells us when the indexer is full.
   * @return true = indexer is full.
   */
  public boolean getIsIndexFull() {
    return isIndexFull;
  } // true = four carrots in indexer {}

  /**
   * Loads carrot into shooter.
   * @param speed The speed of the motor when loading.
   */
  public void loadCarrot(int speed) {
  }

  /**
   * Tells us if the carrot is loaded.
   * @return true = carrot is loaded.
   */
  public boolean getIsCarrotLoaded() {
    return isCarrotLoaded;
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
