// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.trajectory.ExponentialProfile.State;
import edu.wpi.first.wpilibj.CAN;

/** Add your docs here. */
public class Devices {

    //canbus
    public static final String CANBUS_NAME_DRIVE = "Default Name";
    public static final String CANBUS_NAME = "rio";

    //Intake
    public static final int INTAKE_MOTOR_ID = 21; //not true ID change later
    public static final SparkMax INTAKE_MOTOR = new SparkMax(INTAKE_MOTOR_ID, MotorType.kBrushless);

    //Shooter
    public static final int SHOOTER_BOTTOM_WHEEL_ID = 19; //faster
    public static final int SHOOTER_TOP_WHEEL_ID = 51; //slower
    public static final int SHOOTER_CANRANGE_ID = 0; //change id later

    public static final TalonFX SHOOTER_BOTTOM_WHEEL = new TalonFX(SHOOTER_BOTTOM_WHEEL_ID, CANBUS_NAME);
    public static final TalonFX SHOOTER_TOP_WHEEL = new TalonFX(SHOOTER_TOP_WHEEL_ID, CANBUS_NAME);
    
    public static final CANrange SHOOTER_CANRANGE = new CANrange(SHOOTER_CANRANGE_ID, CANBUS_NAME);

    //Indexer
    public static final int INDEXER_ENTERANCE_MOTOR_ID = 16; 
    public static final int INDEXER_ENTERANCE_SENSOR_ID = 0; //not true ID change later
    public static final int INDEXER_EXIT_MOTOR_ID = 5; 
    public static final int INDEXER_EXIT_SENSOR_ID = 0; //not true ID change later

    public static final TalonFX INDEXER_ENTERANCE_MOTOR = new TalonFX(16, CANBUS_NAME);
    public static final TalonFX INDEXER_EXIT_MOTOR = new TalonFX(5, CANBUS_NAME);

    public static final CANrange INDEXER_ENTERANCE_CANRANGE = new CANrange(INDEXER_ENTERANCE_SENSOR_ID, CANBUS_NAME); //change id later
    public static final CANrange INDEXER_EXIT_CANRANGE = new CANrange(INDEXER_EXIT_SENSOR_ID, CANBUS_NAME); //change id later
    
}