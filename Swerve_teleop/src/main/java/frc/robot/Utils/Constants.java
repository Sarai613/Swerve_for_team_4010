package frc.robot.Utils;

import edu.wpi.first.math.geometry.Translation2d;

public class Constants {
    // Max velocity that the motors can reach in meters per second
    public static final float MAX_VELOCITY = 25;
    public static final double DISTANCE_BETWEEN_WHEELS_AND_THE_CENTER = 0.381;

    // Locations for the swerve drive modules relative to the robot center.
    public static final Translation2d FRONT_LEFT_LOCATION = new Translation2d(DISTANCE_BETWEEN_WHEELS_AND_THE_CENTER, DISTANCE_BETWEEN_WHEELS_AND_THE_CENTER);
    public static final Translation2d FRONT_RIGHT_LOCATION = new Translation2d(DISTANCE_BETWEEN_WHEELS_AND_THE_CENTER, -DISTANCE_BETWEEN_WHEELS_AND_THE_CENTER);
    public static final Translation2d BACK_LEFT_LOCATION = new Translation2d(-DISTANCE_BETWEEN_WHEELS_AND_THE_CENTER, DISTANCE_BETWEEN_WHEELS_AND_THE_CENTER);
    public static final Translation2d BACK_RIGHT_LOCATION = new Translation2d(-DISTANCE_BETWEEN_WHEELS_AND_THE_CENTER, -DISTANCE_BETWEEN_WHEELS_AND_THE_CENTER);
    public static final Translation2d ORIGIN = new Translation2d(0, 0);

    public static final double MAX_RADIANS_PER_SECOND = DISTANCE_BETWEEN_WHEELS_AND_THE_CENTER / MAX_VELOCITY * 2 * Math.PI;
}
