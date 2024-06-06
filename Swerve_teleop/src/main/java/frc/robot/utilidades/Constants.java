package frc.robot.utilidades;

import edu.wpi.first.math.geometry.Translation2d;

public class Constants {
    public static final double MAX_SPEED = 20; // in meters per second
    public static final double DISTANCE_BETWEEN_WHEELS_AND_THE_CENTER = 0.381;
    public static final double WHEEL_DIAMETER = 20; //in mm
    public static final double DISTANCE_PER_ROTATION = Math.PI * WHEEL_DIAMETER * 1000; //in meters
    public static final double TURNING_ENCODER_ROT_2_RAD = 2 * Math.PI;
    public static final double TURNING_ENCODER_RPM_2_RAD_PER_SECOND = 1 * Math.PI;
    public static final double P = 0.5; //PID P value
    public static final double JOYSTICK_DEADZONE = 0.05;//this value most be beetwen 0 and 1


    // Locations for the swerve drive modules relative to the robot center.
    public static final Translation2d FRONT_LEFT_LOCATION = new Translation2d(DISTANCE_BETWEEN_WHEELS_AND_THE_CENTER, DISTANCE_BETWEEN_WHEELS_AND_THE_CENTER);
    public static final Translation2d FRONT_RIGHT_LOCATION = new Translation2d(DISTANCE_BETWEEN_WHEELS_AND_THE_CENTER, -DISTANCE_BETWEEN_WHEELS_AND_THE_CENTER);
    public static final Translation2d BACK_LEFT_LOCATION = new Translation2d(-DISTANCE_BETWEEN_WHEELS_AND_THE_CENTER, DISTANCE_BETWEEN_WHEELS_AND_THE_CENTER);
    public static final Translation2d BACK_RIGHT_LOCATION = new Translation2d(-DISTANCE_BETWEEN_WHEELS_AND_THE_CENTER, -DISTANCE_BETWEEN_WHEELS_AND_THE_CENTER);
    public static final Translation2d ORIGIN = new Translation2d(0, 0);
}
