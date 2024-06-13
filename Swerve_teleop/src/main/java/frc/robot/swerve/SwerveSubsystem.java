package frc.robot.swerve;

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
import frc.robot.utilidades.HardwareMap;

public class SwerveSubsystem extends SubsystemBase{
    private final AHRS gyro = new AHRS(SPI.Port.kMXP);
    private final SwerveModulePosition[] swerve_module_position = new SwerveModulePosition[]{
        HardwareMap.frontLeft.getPosition(),
        HardwareMap.frontRight.getPosition(),
        HardwareMap.backLeft.getPosition(),
        HardwareMap.backRight.getPosition()
        };
    
    public final SwerveModuleState[] swerve_module_states = new SwerveModuleState[]{
        HardwareMap.frontLeft.getState(),
        HardwareMap.frontRight.getState(),
        HardwareMap.backLeft.getState(),
        HardwareMap.backRight.getState()
        };;
    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(Chassis.robot_kinematics,
            new Rotation2d(0), swerve_module_position,
            new Pose2d(0, 0, Rotation2d.fromDegrees(0)));

    
    public SwerveSubsystem() {
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
        swerve_module_states[0] = HardwareMap.frontLeft.getState();
        swerve_module_states[1] = HardwareMap.frontRight.getState();
        swerve_module_states[2] = HardwareMap.backLeft.getState();
        swerve_module_states[3] = HardwareMap.backRight.getState();
    }
    public void updateModulePosition(){
        swerve_module_position[0] = HardwareMap.frontLeft.getPosition();
        swerve_module_position[1] = HardwareMap.frontRight.getPosition();
        swerve_module_position[2] = HardwareMap.backLeft.getPosition();
        swerve_module_position[3] = HardwareMap.backRight.getPosition();
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
        HardwareMap.frontLeft.stop();
        HardwareMap.frontRight.stop();
        HardwareMap.backLeft.stop();
        HardwareMap.backRight.stop();
    }

    // Set the desired state for each swerveModule by giving an array of states
    public void setStates(SwerveModuleState[] desired_states){
        SwerveDriveKinematics.desaturateWheelSpeeds(desired_states, Constants.MAX_SPEED);
        HardwareMap.frontLeft.setDesiredState(desired_states[2]);
        HardwareMap.frontRight.setDesiredState(desired_states[3]);
        HardwareMap.backLeft.setDesiredState(desired_states[0]);
        HardwareMap.backRight.setDesiredState(desired_states[1]);
    }
}
