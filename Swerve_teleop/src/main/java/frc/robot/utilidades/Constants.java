package frc.robot.utilidades;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class Constants {
    public static final double WHEEL_DIAMETER = 50.17 * 2; //in mm
    public static final double DISTANCE_BETWEEN_WHEELS_AND_THE_CENTER = .28; //in meters
    public static final double DRIVE_MOTOR_GEAR_RATIO = 1/8.14;
    public static final double RPM = 5676;
    public static final double TURNING_MOTOR_GEAR_RATIO = 1/12.8;
    public static final double MAX_SPEED = RPM / 60 * DRIVE_MOTOR_GEAR_RATIO * WHEEL_DIAMETER * Math.PI * 1000; // in m/s
    public static final double MAX_ANGULAR_SPEED = RPM * TURNING_MOTOR_GEAR_RATIO / 60 * 2 * Math.PI; // in radians per second
    public static final double DRIVE_ENCODER_ROT_2_METER = DRIVE_MOTOR_GEAR_RATIO * Math.PI * WHEEL_DIAMETER;
    public static final double DRIVE_ENCODER_RPM_2_METERS_PER_SECOND = DRIVE_ENCODER_ROT_2_METER / 60; //in meters
    public static final double TURNING_ENCODER_ROT_2_RAD = TURNING_MOTOR_GEAR_RATIO * 2 * Math.PI;
    public static final double TURNING_ENCODER_RPM_2_RAD_PER_SECOND = TURNING_ENCODER_ROT_2_RAD / 60;
    public static final double P = 0.217; //PID P value
    public static final double JOYSTICK_DEADZONE = 0.09;//this value most be beetwen 0 and 1


    // Locations for the swerve drive modules relative to the robot center.
    public static final Translation2d FRONT_LEFT_LOCATION = new Translation2d(DISTANCE_BETWEEN_WHEELS_AND_THE_CENTER, DISTANCE_BETWEEN_WHEELS_AND_THE_CENTER);
    public static final Translation2d FRONT_RIGHT_LOCATION = new Translation2d(DISTANCE_BETWEEN_WHEELS_AND_THE_CENTER, -DISTANCE_BETWEEN_WHEELS_AND_THE_CENTER);
    public static final Translation2d BACK_LEFT_LOCATION = new Translation2d(-DISTANCE_BETWEEN_WHEELS_AND_THE_CENTER, DISTANCE_BETWEEN_WHEELS_AND_THE_CENTER);
    public static final Translation2d BACK_RIGHT_LOCATION = new Translation2d(-DISTANCE_BETWEEN_WHEELS_AND_THE_CENTER, -DISTANCE_BETWEEN_WHEELS_AND_THE_CENTER);
    public static final Translation2d ORIGIN = new Translation2d(0, 0);

    // Autonomous constants
    public static double AUTONOMOUS_P_X = .35;
    public static double AUTONOMOUS_I_X = 0;
    public static double AUTONOMOUS_D_X = 0;
    public static double AUTONOMOUS_P_Y = .35;
    public static double AUTONOMOUS_I_Y = 0;
    public static double AUTONOMOUS_D_Y = 0;
    public static double AUTONOMOUS_P_Z = .35;
    public static double AUTONOMOUS_I_Z = 0;
    public static double AUTONOMOUS_D_Z = 0;
    public static double AUTONOMOUS_MAX_ACCELERATION = 1;
    public static double AUTONOMOUS_MAX_SPEED = MAX_SPEED * 0.2;
    public static final double AUTONOMOUS_MAX_ANGULAR_ACCELERATION = .1;
    public static final double AUTONOMOUS_MAX_ANGULAR_SPEED = MAX_ANGULAR_SPEED * .1;

    // The TrapezoidProfile is used for stablish a limit for the angular speed and acceleration of the turning motor
    public static TrapezoidProfile.Constraints AUTONOMOUS_Z_CONSTRAIT = 
        new TrapezoidProfile.Constraints(
            AUTONOMOUS_MAX_ANGULAR_SPEED,
            AUTONOMOUS_MAX_ANGULAR_ACCELERATION);
}
