package frc.robot.utils;

import edu.wpi.first.math.geometry.Translation2d;

public class Constants {
    // Max velocity that the motors can reach in meters per second
    public static final double MAX_SPEED = 20;
    public static final double DISTANCE_BETWEEN_WHEELS_AND_THE_CENTER = 0.381; 
    public static final double WHEEL_RADIUS = 0.0508;
    public static final int ENCODER_RESOLUTION = 4096;
    public static final double MAX_ANGULAR_VELOCITY = 20;
    public static final double MAX_ANGULAR_ACCELERATION = 2 * Math.PI; // radians per second squared

    // Locations for the swerve drive modules relative to the robot center.
    public static final Translation2d FRONT_LEFT_LOCATION = new Translation2d(DISTANCE_BETWEEN_WHEELS_AND_THE_CENTER, DISTANCE_BETWEEN_WHEELS_AND_THE_CENTER);
    public static final Translation2d FRONT_RIGHT_LOCATION = new Translation2d(DISTANCE_BETWEEN_WHEELS_AND_THE_CENTER, -DISTANCE_BETWEEN_WHEELS_AND_THE_CENTER);
    public static final Translation2d BACK_LEFT_LOCATION = new Translation2d(-DISTANCE_BETWEEN_WHEELS_AND_THE_CENTER, DISTANCE_BETWEEN_WHEELS_AND_THE_CENTER);
    public static final Translation2d BACK_RIGHT_LOCATION = new Translation2d(-DISTANCE_BETWEEN_WHEELS_AND_THE_CENTER, -DISTANCE_BETWEEN_WHEELS_AND_THE_CENTER);
    public static final Translation2d ORIGIN = new Translation2d(0, 0);
}
