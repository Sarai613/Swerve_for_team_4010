package frc.robot.swerve;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.ExponentialProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utilidades.Constants;

public class SwerveModule {

    private final CANSparkMax drive_motor;
    private final CANSparkMax turning_motor;
    private final CANEncoder drive_encoder;
    private final CANEncoder turning_encoder;
    private final PIDController turning_PID_controller;
    private final int drive_spark_id;
    private final int turning_spark_id;

    public SwerveModule(int drive_spark_id, int turning_spark_id) {
        this.drive_spark_id = drive_spark_id;
        this.turning_spark_id = turning_spark_id;

        drive_motor = new CANSparkMax(drive_spark_id, MotorType.kBrushless);
        turning_motor = new CANSparkMax(turning_spark_id, MotorType.kBrushless);

        drive_encoder = drive_motor.getEncoder();
        turning_encoder = turning_motor.getEncoder();
        
        drive_encoder.setPositionConversionFactor(Constants.DISTANCE_PER_ROTATION);
        drive_encoder.setVelocityConversionFactor(Constants.MAX_SPEED);
        turning_encoder.setPositionConversionFactor(Constants.TURNING_ENCODER_ROT_2_RAD);
        turning_encoder.setVelocityConversionFactor(Constants.TURNING_ENCODER_RPM_2_RAD_PER_SECOND);

        turning_PID_controller = new PIDController(Constants.P, 0, 0);
        turning_PID_controller.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();
    }

    public SwerveModuleState getState() {
        double driveSpeed = drive_encoder.getVelocity();
        double turningPosition = turning_encoder.getPosition();

        return new SwerveModuleState(driveSpeed, new Rotation2d(turningPosition));
    }

    public SwerveModulePosition getPosition() {
        double driveDistance = drive_encoder.getPosition();
        double turningPosition = turning_encoder.getPosition();

        return new SwerveModulePosition(driveDistance, new Rotation2d(turningPosition));
    }

    public void resetEncoders(){
        drive_encoder.setPosition(0);
        turning_encoder.setPosition(0);
    }

    public void stop(){
        drive_motor.set(0);
        turning_motor.set(0);
    }

    public void setDesiredState(SwerveModuleState desiredState) {

        if (Math.abs(desiredState.speedMetersPerSecond) < Constants.JOYSTICK_DEADZONE){
            stop();
            return;
        }

        var encoder_rotation = new Rotation2d(turning_encoder.getPosition());

        // Optimiza el estado de referencia para evitar giros mayores a 90 grados
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, encoder_rotation);

        // Escala la velocidad por el coseno del error angular
        state.speedMetersPerSecond *= state.angle.minus(encoder_rotation).getCos();
        
        double turning_motor_position = turning_encoder.getPosition();
        drive_motor.set(state.speedMetersPerSecond / Constants.MAX_SPEED);
        turning_motor.set(turning_PID_controller.calculate(turning_motor_position, state.angle.getRadians()));
        SmartDashboard.putString("SwerveModule[" + drive_spark_id + turning_spark_id + "] state: " + state.toString());


        
    }
}
