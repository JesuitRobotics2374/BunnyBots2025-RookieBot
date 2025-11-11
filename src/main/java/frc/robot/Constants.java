package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class Constants {
    // Critical Generic Constants
    public static final double MAX_SPEED = 0.90; // kSpeedAt12Volts desired top speed
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

    //public static final AprilTagFields FIELD_LAYOUT = AprilTagFields.k2025ReefscapeWelded;
    public static final String FIELD_PATH = "BunnyBots2025Field.json";
  
    public static final Pose2d TEST_PATHFIND_TARGET = new Pose2d(1.199, 7.028, new Rotation2d(128.581 * (Math.PI / 180)));

    // Pathfinding
    //public static final double PATHFINDING_MAX_VELOCITY = 3.5;
    public static final double PATHFINDING_MAX_VELOCITY = .5;
    public static final double PATHFINDING_MAX_ACCELERATION = 1;
    public static final double PATHFINDING_MAX_ROTATIONAL_VELOCITY = Units.degreesToRadians(540);
    public static final double PATHFINDING_MAX_ROTATIONAL_ACCELERATION = Units.degreesToRadians(720);
    
    public static final double PATHFINDING_PRE_BUFFER = -1.60116; // meters
    public static final double PATHFINDING_POST_BUFFER = -0.279; // meters
    public static final double PATHFINDING_FRONT_BUFFER = -1.02; // meters
    public static final double PATHFINDING_LEFT_SHIFT_FACTOR = -0.2727;
    public static final double PATHFINDING_RIGHT_SHIFT_FACTOR = 0.1437;

    public static final double GENERIC_DISTANCE_THRESHOLD = 0.035; 
    public static final double GENERIC_ROTATION_THRESHOLD = 0.8 * Math.PI / 180;
    public static final double ALIGN_Y_SHIFT = -0.12; //meters limelight
    public static final double ALIGN_MOVE_SPEED = 0.25;
    public static final double ALIGN_ROTATE_SPEED = 0.0006;
    public static final double ALIGN_ROTATIONAL_FEED_FORWARD = 0.25;

    public static final int PIGEON_ID = 0;
}