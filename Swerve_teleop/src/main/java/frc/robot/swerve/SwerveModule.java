package frc.robot.swerve;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utilidades.Constants;

public class SwerveModule {

    private final CANSparkMax drive_motor;
    private final CANSparkMax turning_motor;
    private final RelativeEncoder drive_encoder;
    private final RelativeEncoder turning_encoder;
    private final PIDController turning_PID_controller;
    private final int drive_spark_id;
    private final int turning_spark_id;
    private final AnalogInput absolute_encoder;
    private final double absolute_encoder_offset;


    public SwerveModule(int drive_spark_id, int turning_spark_id, int absolute_encoder_id, double absolute_encoder_offset) {
        this.drive_spark_id = drive_spark_id;
        this.turning_spark_id = turning_spark_id;

        this.absolute_encoder_offset = absolute_encoder_offset;
        absolute_encoder = new AnalogInput(absolute_encoder_id);

        drive_motor = new CANSparkMax(drive_spark_id, MotorType.kBrushless);
        turning_motor = new CANSparkMax(turning_spark_id, MotorType.kBrushless);

        drive_encoder = drive_motor.getEncoder();
        turning_encoder = turning_motor.getEncoder();
        
        drive_encoder.setPositionConversionFactor(Constants.DRIVE_ENCODER_RPM_2_METERS_PER_SECOND);
        drive_encoder.setVelocityConversionFactor(Constants.DRIVE_ENCODER_ROT_2_METER);
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
        turning_encoder.setPosition(getAbsoluteEncoderRad());
    }

    public void stop(){
        drive_motor.set(0);
        turning_motor.set(0);
    }

    public double getAbsoluteEncoderRad(){
        double angle = absolute_encoder.getVoltage() / RobotController.getVoltage5V();
        angle *= 2 * Math.PI;
        angle -= absolute_encoder_offset;
        return angle;
    }

    public void setDesiredState(SwerveModuleState desiredState) {

        if (Math.abs(desiredState.speedMetersPerSecond) < 0.09){
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
        SmartDashboard.putString("Debug", "SwerveModule[" + Integer.toString(drive_spark_id) + ", " + Integer.toString(turning_spark_id) + "] state: " + state.toString());


        
    }
}
