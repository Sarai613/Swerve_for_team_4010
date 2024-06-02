package frc.robot.teleop.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import frc.robot.Utils.Constants;

public class Chassis {

    public boolean field_oriented = true;
    public int robot_turning_encoder = 0;
    //making an instance of ChassisSpeeds for making the movement calculation easier
    public ChassisSpeeds speeds;

    // Creating kinematics object using the module locations
    SwerveDriveKinematics robot_kinematics = new SwerveDriveKinematics(
    Constants.FRONT_LEFT_LOCATION, Constants.FRONT_RIGHT_LOCATION, Constants.BACK_LEFT_LOCATION, Constants.BACK_RIGHT_LOCATION
    );

    // Get the rotation of the robot from the gyro.
      // The gyro sensor
    private final ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();
    public Rotation2d gyroAngle = m_gyro.getRotation2d();

    // Convert to module states
    public SwerveModuleState[] moduleStates = robot_kinematics.toSwerveModuleStates(speeds);

    public Chassis(boolean field_oriented){
        this.field_oriented = field_oriented;

    }

    public SwerveModuleState[] updateModuleStateWithPercentages(float x, float y, float z){
        x *= Constants.MAX_VELOCITY;
        y *= Constants.MAX_VELOCITY;
        z *= Constants.MAX_RADIANS_PER_SECOND;

        // Establishes the speed variable depending on the field_oriented
        if(field_oriented) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                2.0, 2.0, Math.PI / 2.0, Rotation2d.fromDegrees(45.0));
        } else {
            speeds = new ChassisSpeeds(0, 0, 0);
        }

        speeds.vxMetersPerSecond = x;
        speeds.vyMetersPerSecond = y;
        speeds.omegaRadiansPerSecond = z;

        // Convert to module states
        moduleStates = robot_kinematics.toSwerveModuleStates(speeds);

        // Front left module state
        SwerveModuleState frontLeft = moduleStates[0];

        // Front right module state
        SwerveModuleState frontRight = moduleStates[1];

        // Back left module state
        SwerveModuleState backLeft = moduleStates[2];

        // Back right module state
        SwerveModuleState backRight = moduleStates[3];

        SwerveModuleState frontLeftOptimized = SwerveModuleState.optimize(frontLeft,
            gyroAngle);
        
        SwerveModuleState frontRightOptimized = SwerveModuleState.optimize(frontRight,
            gyroAngle);
        
        SwerveModuleState backLeftOptimized = SwerveModuleState.optimize(backLeft,
            gyroAngle);
        
        SwerveModuleState backRightOptimized = SwerveModuleState.optimize(backRight,
            gyroAngle);

        moduleStates = new SwerveModuleState[]{
            frontLeftOptimized, 
            frontRightOptimized,
            backLeftOptimized,
            backRightOptimized
        };

        return moduleStates;
    }
}
