package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class Constants {

    final public static double CONVEYOR_TARGET_SPEED = 0.2; // wth formatting this like we did with shooter gives errors
  
    public final static double TARGET_SHOOTER_SPEED = 0.2; // change later
    public final static double SHOOTER_MAX_RPM = 6000; //incorrect, change later 
  
    // Critical Generic Constants
    public static final double MAX_SPEED = 0.475; // kSpeedAt12Volts desired top speed
    public static final double MAX_SPEED_TURBO = 0.65;
    public static final double MAX_ANGULAR_RATE = 0.45; // 3/4 of a rotation per second max angular velocity

    public static final double FIELD_X_MIDPOINT = 0; // 8.779; // meters
    public static final double FIELD_Y_MIDPOINT = 0; // 4.026; // meters

    // General Constants
    public static final int SENSOR_PORT = 18;
    public static final String DRIVER_READOUT_TAB_NAME = "Driver Readout";

    // PhotonVision
    public static final int numberOfCams = 2;

    public static final double MIN_CAMERA_DISTANCE = 0; // meters TODO

    public static final AprilTagFields FIELD_LAYOUT = AprilTagFields.k2025ReefscapeWelded;
  
    public static final Pose2d TEST_PATHFIND_TARGET = new Pose2d(1.199, 7.028, new Rotation2d(128.581 * (Math.PI / 180)));
    
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
