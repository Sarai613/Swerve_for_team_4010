package frc.robot.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.utilidades.Constants;
import frc.robot.utilidades.HardwareMap;

public class Chassis {

    public int robot_turning_encoder = 0;
    //making an instance of ChassisSpeeds for making the movement calculation easier
    public ChassisSpeeds speeds;
    SwerveSubsystem odometry = new SwerveSubsystem();

    // Creating kinematics object using the module locations
    public static SwerveDriveKinematics robot_kinematics = new SwerveDriveKinematics(
    Constants.FRONT_LEFT_LOCATION, Constants.FRONT_RIGHT_LOCATION, Constants.BACK_LEFT_LOCATION, Constants.BACK_RIGHT_LOCATION
    );

    // Get the rotation of the robot from the gyro.
      // The gyro sensor
    private Rotation2d gyroAngle = odometry.getRotation2d();

    // Convert to module states
    public SwerveModuleState[] moduleStates;

    public void drive(double x, double y, double z,boolean field_oriented){
        gyroAngle = odometry.getRotation2d();

        // Establishes the speed variable depending on the field_oriented
        if(field_oriented) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                x, y, z, gyroAngle);
        } else {
            speeds = new ChassisSpeeds(x, y, z);
        }

        speeds.vxMetersPerSecond = x;
        speeds.vyMetersPerSecond = y;
        speeds.omegaRadiansPerSecond = z;

        moduleStates = robot_kinematics.toSwerveModuleStates(speeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.MAX_SPEED);
        HardwareMap.frontLeft.setDesiredState(moduleStates[0]);
        HardwareMap.frontRight.setDesiredState(moduleStates[1]);
        HardwareMap.backLeft.setDesiredState(moduleStates[2]);
        HardwareMap.backRight.setDesiredState(moduleStates[3]);
    }

    
}
