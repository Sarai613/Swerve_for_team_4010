package frc.robot.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import frc.robot.utilidades.HardwareMap;

public class OdometrySwerve {
    public final ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();

    Pose2d initial_pose;
    public SwerveDriveOdometry robot_odometry;
    Chassis chassis;
    public SwerveModuleState[] real_module_states = {
        HardwareMap.frontLeft.getState(),
        HardwareMap.frontRight.getState(),
        HardwareMap.backLeft.getState(),
        HardwareMap.backRight.getState(),
    };

    public void initialize(){
        robot_odometry = new SwerveDriveOdometry(
            chassis.robot_kinematics, m_gyro.getRotation2d(),
            new SwerveModulePosition[] {
                HardwareMap.frontLeft.getPosition(),
                HardwareMap.frontRight.getPosition(),
                HardwareMap.backLeft.getPosition(),
                HardwareMap.backRight.getPosition()
            }, initial_pose);
    }

    public OdometrySwerve(Chassis chassis, Pose2d initial_pose){
        this.chassis = chassis;
        this.initial_pose = initial_pose;
        initialize();
    }

    public void updateOdometry(){
        robot_odometry.update(
            m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
            HardwareMap.frontLeft.getPosition(),
            HardwareMap.frontRight.getPosition(),
            HardwareMap.backLeft.getPosition(),
            HardwareMap.backRight.getPosition()
        });
    }

    
}
