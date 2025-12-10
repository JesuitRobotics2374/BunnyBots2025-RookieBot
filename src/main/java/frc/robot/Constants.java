package frc.robot;

import java.nio.file.Paths;

import edu.wpi.first.wpilibj.Filesystem;

public class Constants {

    public static final Core.AutonomousPlan AUTO_PLAN = Core.AutonomousPlan.NO_AUTO;

    public static final double AUTO_STATIC_MOVE_SPEED = 0.5;

    public static final double AUTO_X_OFFSET = 1;
    public static final double AUTO_Y_OFFSET = 0;
    public static final double AUTO_Z_OFFSET = 0;
    
    public static final double AUTO_ROLL_OFFSET = 0;
    public static final double AUTO_PITCH_OFFSET = 0;
    public static final double AUTO_YAW_OFFSET = 0;
  
    // public static final double INTAKE_RETRACTED_POSITION = 0;
    // public static final double INTAKE_DEPLOYED_POSITION = 0.3;

    public static final double CONVEYER_ENTRANCE_SPEED = 0.1;
    public static final double CONVEYER_EXIT_SPEED = 1;
  
    public static final double TARGET_BOTTOM_SHOOTER_SPEED = 0.25; // change later
    public static final double TARGET_TOP_SHOOTER_SPEED = 0.25 * (0.5 / 0.3);
    public static final double SHOOTER_MAX_RPM = 6000; // dont think we're using this
  
    // Critical Generic Constants
    public static final double MAX_SPEED = 0.475; // kSpeedAt12Volts desired top speed
    public static final double MAX_SPEED_TURBO = 0.65;
    public static final double MAX_ANGULAR_RATE = 1.1; // 3/4 of a rotation per second max angular velocity

    public static final double FIELD_X_MIDPOINT = 0; // 8.779; // meters
    public static final double FIELD_Y_MIDPOINT = 0; // 4.026; // meters

    // General Constants
    public static final int SENSOR_PORT = 18;
    public static final String DRIVER_READOUT_TAB_NAME = "Driver Readout";

    // PhotonVision
    public static final int numberOfCams = 2;

    public static final String FIELD_PATH = Paths.get(Filesystem.getDeployDirectory().getAbsolutePath(), "BunnyBots2025Field.json").toString();
  
    // CAN RANGE Movement - Weird Units
    public static final double RIGHT_CANRANGE_OFFSET = -0.04;

    public static final double CAN_RANGE_SPEED = 0.6;
    public static final double CAN_RANGE_FORWARD_DISTANCE = 0.77;
    public static final double CAN_RANGE_BACKWARD_DISTANCE = 1.3;
    
    public static final double SA_RIGHT_BUFFER = -0.0;
    public static final double SA_TARGET_DISTANCE = 0.738;
    public static final double SA_ROTATIONAL_RATE_THRESHOLD = 0.04;

    public static final int PIGEON_ID = 0;
    public static final double MAX_TIP_ANGLE = 8.0;
}
