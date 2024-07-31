package frc.robot.susbsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilidades.Constants;
import frc.robot.utilidades.SwerveModule;

public class Swerve extends SubsystemBase{

    //Defines every single module by giving the drive spark id, the turning spark id, the absolute encoder id, absolute encoder offset, is inverted
    private final SwerveModule frontLeft = new SwerveModule(17, 33, 11,0, false);
    private final SwerveModule frontRight = new SwerveModule(31, 22, 10,0, true);
    private final SwerveModule backLeft = new SwerveModule(16, 18, 13, 0, true);
    private final SwerveModule backRight = new SwerveModule(21, 34, 12,  0, false);

    private final AHRS gyro = new AHRS(SPI.Port.kMXP);

    public int robot_turning_encoder = 0;

    // Creating kinematics object using the module locations
    public SwerveDriveKinematics robot_kinematics = new SwerveDriveKinematics(
    Constants.FRONT_LEFT_LOCATION, Constants.FRONT_RIGHT_LOCATION, Constants.BACK_LEFT_LOCATION, Constants.BACK_RIGHT_LOCATION
    );

    // Convert to module states
    public SwerveModuleState[] moduleStates;

    private final SwerveModulePosition[] swerve_module_position = new SwerveModulePosition[]{
        frontLeft.getPosition(),
        frontRight.getPosition(),
        backLeft.getPosition(),
        backRight.getPosition()
        };
    
    public final SwerveModuleState[] swerve_module_states = new SwerveModuleState[]{
        frontLeft.getState(),
        frontRight.getState(),
        backLeft.getState(),
        backRight.getState()
        };;
    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(robot_kinematics,
            new Rotation2d(0), swerve_module_position,
            new Pose2d(0, 0, Rotation2d.fromDegrees(0)));

    
    public Swerve() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading(); // Reset the gyroscope When the robot is initialized
            } catch (Exception e) {
            }
        }).start();
    }

    // Reset the gyroscope
    public void zeroHeading() {
        gyro.reset();
    }

    // Returns the actual robot angle
    public double getHeading() {
        return Math.IEEEremainder(-gyro.getAngle(), 360);
    }

    // Returns a Rotation2d class with the robot angle
    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
        //return Rotation2d.fromDegrees(36);
    }

    // Return actual robot position
    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    // Resets the Odometer
    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(getRotation2d(), swerve_module_position, pose);
    }

    // Updates the Odometer
    public void updateOdometry(){
        updateModulePosition();
        odometer.update(getRotation2d(), swerve_module_position);
    }
    
    // Update the module states reading
    public void updateModuleStates(){
        swerve_module_states[0] = frontLeft.getState();
        swerve_module_states[1] = frontRight.getState();
        swerve_module_states[2] = backLeft.getState();
        swerve_module_states[3] = backRight.getState();
    }
    public void updateModulePosition(){
        swerve_module_position[0] = frontLeft.getPosition();
        swerve_module_position[1] = frontRight.getPosition();
        swerve_module_position[2] = backLeft.getPosition();
        swerve_module_position[3] = backRight.getPosition();
    }

    @Override
    // This repeat periodically during the subsystem use
    public void periodic() {
        updateOdometry();
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
    }

    // Stop the swerve modules
    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    // Set the desired state for each swerveModule by giving an array of states
    public void setStates(SwerveModuleState[] desired_states){
        SwerveDriveKinematics.desaturateWheelSpeeds(desired_states, Constants.MAX_SPEED);
        frontLeft.setDesiredState(desired_states[2]);
        frontRight.setDesiredState(desired_states[3]);
        backLeft.setDesiredState(desired_states[0]);
        backRight.setDesiredState(desired_states[1]);
    }
}
