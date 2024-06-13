package frc.robot.swerve;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utilidades.Constants;
import frc.robot.utilidades.HardwareMap;

public class Chassis {

    public int robot_turning_encoder = 0;

    SwerveSubsystem odometry = new SwerveSubsystem();

    // Creating kinematics object using the module locations
    public static SwerveDriveKinematics robot_kinematics = new SwerveDriveKinematics(
    Constants.FRONT_LEFT_LOCATION, Constants.FRONT_RIGHT_LOCATION, Constants.BACK_LEFT_LOCATION, Constants.BACK_RIGHT_LOCATION
    );

    // Convert to module states
    public SwerveModuleState[] moduleStates;


    // This function uses the setDesiredState function in swerve module to indicate what should be the state. 
    // For this we use .toSwerveModuleStates function in the kinematics that convert a ChassisSpeeds class
    // into an array of ModuleStates
    public void drive(double x, double y, double z, boolean field_oriented, SwerveSubsystem subsystem){

        //making an instance of ChassisSpeeds for making the movement calculation easier
        ChassisSpeeds speeds;

        // Establishes the speed variable depending on the field_oriented
        if(field_oriented) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                x, y, z, odometry.getRotation2d());
            SmartDashboard.putString("angle", "angle: " + odometry.getRotation2d());
        } else {
            speeds = new ChassisSpeeds(x, y, z);
        }

        moduleStates = robot_kinematics.toSwerveModuleStates(speeds);

        subsystem.setStates(moduleStates);
    }

    
}
