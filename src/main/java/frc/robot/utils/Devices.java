// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.core.CoreCANrange;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

/** Add your docs here. */
public class Devices {

    //Intake
    public static final SparkMax INTAKE_MOTOR = new SparkMax(32, MotorType.kBrushless);

    //Shooter
    public static final int SHOOTER_BOTTOM_WHEEL_ID = 19; //faster
    public static final int SHOOTER_TOP_WHEEL_ID = 51; //slower
    public static final int SHOOTER_CANRANGE_ID = 17; //change id later

    public static final TalonFX SHOOTER_BOTTOM_WHEEL = new TalonFX(SHOOTER_BOTTOM_WHEEL_ID, "rio");
    public static final TalonFX SHOOTER_TOP_WHEEL = new TalonFX(SHOOTER_TOP_WHEEL_ID, "rio");
    
    public static final CoreCANrange SHOOTER_CANRANGE = new CoreCANrange(SHOOTER_CANRANGE_ID, "rio");

    //Indexer
    public static final int INDEXER_ENTERANCE_MOTOR_ID = 16; 
    public static final int INDEXER_ENTERANCE_SENSOR_ID = 19; //not true ID change later
    public static final int INDEXER_EXIT_MOTOR_ID = 5; 
    public static final int INDEXER_EXIT_SENSOR_ID = 27; //not true ID change later

    public static final TalonFX INDEXER_ENTERANCE_MOTOR = new TalonFX(16, "rio");
    public static final TalonFX INDEXER_EXIT_MOTOR = new TalonFX(5, "rio");

    public static final CoreCANrange INDEXER_ENTERANCE_CANRANGE = new CoreCANrange(INDEXER_ENTERANCE_SENSOR_ID, "rio"); //change id later
    public static final CoreCANrange INDEXER_EXIT_CANRANGE = new CoreCANrange(INDEXER_EXIT_SENSOR_ID, "rio"); //change id later
    
}