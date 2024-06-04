package frc.robot.swerve;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {

    private final CANSparkMax m_driveMotor;
    private final CANSparkMax m_turningMotor;
    private final RelativeEncoder m_driveEncoder;
    private final RelativeEncoder m_turningEncoder;
    private final SparkPIDController m_drivePIDController;
    private final SparkPIDController m_turningPIDController;

    public SwerveModule(int drive_spark_id, int turning_spark_id) {
        m_driveMotor = new CANSparkMax(drive_spark_id, MotorType.kBrushless);
        m_turningMotor = new CANSparkMax(turning_spark_id, MotorType.kBrushless);

        m_driveEncoder = m_driveMotor.getEncoder();
        m_turningEncoder = m_turningMotor.getEncoder();

        m_drivePIDController = m_driveMotor.getPIDController();
        m_turningPIDController = m_turningMotor.getPIDController();

        // Ajustar PID para el motor de tracción
        m_drivePIDController.setP(0.1);  // Valor de ejemplo, ajustar según sea necesario
        m_drivePIDController.setI(0.001);
        m_drivePIDController.setD(1.0);

        // Ajustar PID para el motor de giro
        m_turningPIDController.setP(0.1);  // Valor de ejemplo, ajustar según sea necesario
        m_turningPIDController.setI(0.001);
        m_turningPIDController.setD(1.0);
        
    }

    public SwerveModuleState getState() {
        double driveSpeed = m_driveEncoder.getVelocity();
        double turningPosition = m_turningEncoder.getPosition();

        return new SwerveModuleState(driveSpeed, new Rotation2d(turningPosition));
    }

    public SwerveModulePosition getPosition() {
        double driveDistance = m_driveEncoder.getPosition();
        double turningPosition = m_turningEncoder.getPosition();

        return new SwerveModulePosition(driveDistance, new Rotation2d(turningPosition));
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        var encoderRotation = new Rotation2d(m_turningEncoder.getPosition());

        // Optimiza el estado de referencia para evitar giros mayores a 90 grados
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, encoderRotation);

        // Escala la velocidad por el coseno del error angular
        state.speedMetersPerSecond *= state.angle.minus(encoderRotation).getCos();

        double desired_turning_position = 42 / 360 * state.angle.getDegrees();

        m_drivePIDController.setReference(state.speedMetersPerSecond, ControlType.kVelocity);
        m_turningPIDController.setReference(desired_turning_position, ControlType.kPosition);
    }
}
