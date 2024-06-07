package frc.robot.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.kauailabs.navx.frc.AHRS;
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
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
    }

    public void zeroHeading() {
        gyro.reset();
    }

    public double getHeading() {
        return Math.IEEEremainder(gyro.getAngle(), 360);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(getRotation2d(), swerve_module_position, pose);
    }

    public void updateOdometry(){
        odometer.update(getRotation2d(), swerve_module_position);
    }

    public void updateModuleStates(){
        swerve_module_states[0] = HardwareMap.frontLeft.getState();
        swerve_module_states[1] = HardwareMap.frontRight.getState();
        swerve_module_states[2] = HardwareMap.backLeft.getState();
        swerve_module_states[3] = HardwareMap.backRight.getState();
    }

    @Override
    public void periodic() {
        updateOdometry();
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
    }

    public void stopModules() {
        HardwareMap.frontLeft.stop();
        HardwareMap.frontRight.stop();
        HardwareMap.backLeft.stop();
        HardwareMap.backRight.stop();
    }

    public void setStates(SwerveModuleState[] desired_states){
        SwerveDriveKinematics.desaturateWheelSpeeds(desired_states, Constants.MAX_SPEED);
        HardwareMap.frontLeft.setDesiredState(desired_states[0]);
        HardwareMap.frontRight.setDesiredState(desired_states[1]);
        HardwareMap.backLeft.setDesiredState(desired_states[2]);
        HardwareMap.backRight.setDesiredState(desired_states[3]);
    }
}
