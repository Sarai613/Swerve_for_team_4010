package frc.robot.swerve;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.utilidades.Constants;

public class SwerveModule {

    private final CANSparkMax drive_motor;
    private final CANSparkMax turning_motor;
    private final RelativeEncoder drive_encoder;
    private final RelativeEncoder turning_encoder;
    private final PIDController turning_PID_controller;
    private final int drive_spark_id;
    private final int turning_spark_id;
    private CANcoder absolute_encoder;
    private final double absolute_encoder_offset;


    public SwerveModule(int drive_spark_id, int turning_spark_id, int absolute_encoder_id, double absolute_encoder_offset, Boolean drive_inverted) {
        this.drive_spark_id = drive_spark_id;
        this.turning_spark_id = turning_spark_id;

        this.absolute_encoder_offset = absolute_encoder_offset;
        absolute_encoder = new CANcoder(absolute_encoder_id);

        drive_motor = new CANSparkMax(this.drive_spark_id, MotorType.kBrushless);
        turning_motor = new CANSparkMax(this.turning_spark_id, MotorType.kBrushless);

        drive_encoder = drive_motor.getEncoder();
        turning_encoder = turning_motor.getEncoder();

        drive_motor.setInverted(drive_inverted);
        
        // Converts the encoder rotations into common units like meters per second
        drive_encoder.setPositionConversionFactor(Constants.DRIVE_ENCODER_ROT_2_METER);
        drive_encoder.setVelocityConversionFactor(Constants.DRIVE_ENCODER_RPM_2_METERS_PER_SECOND);
        turning_encoder.setPositionConversionFactor(Constants.TURNING_ENCODER_ROT_2_RAD);
        turning_encoder.setVelocityConversionFactor(Constants.TURNING_ENCODER_RPM_2_RAD_PER_SECOND);

        // Assigns a pid controller for the turning motor. This one takes a P variable stablish in constants that specifies the proportional PID value
        turning_PID_controller = new PIDController(Constants.P, Constants.I, Constants.D);
        turning_PID_controller.enableContinuousInput(-Math.PI, Math.PI);

        // Set The encoders into 0 position
        resetEncoders();
    }

    // Returns a SwerveModuleState object with the module state
    public SwerveModuleState getState() {
        double driveSpeed = drive_encoder.getVelocity();
        double turningPosition = turning_encoder.getPosition();

        return new SwerveModuleState(driveSpeed, new Rotation2d(turningPosition));
    }

    // Returns a SwerveModulePosition object with the actual modules position
    public SwerveModulePosition getPosition() {
        double driveDistance = drive_encoder.getPosition();
        double turningPosition = turning_encoder.getPosition();

        return new SwerveModulePosition(driveDistance, new Rotation2d(turningPosition));
    }

    // Stablish the encoders into 0 position
    public void resetEncoders(){
        drive_encoder.setPosition(0);
        turning_encoder.setPosition(getAbsoluteEncoderRad()); // Calibrate the turning encoder with the absolute encoder
    }

    // Stops the motors
    public void stop(){
        drive_motor.set(0);
        turning_motor.set(0);
    }

    // Returns the absolute encoder actual radians
    public double getAbsoluteEncoderRad(){
        double angle = absolute_encoder.getAbsolutePosition().getValue();
        angle *= 2 * Math.PI;
        angle -= absolute_encoder_offset;
        //SmartDashboard.putString("algo", angle.toString);
        return angle;
    }

    // Move the module by giving a SwerveModuleState object
    public void setDesiredState(SwerveModuleState desiredState) {

        // Avoid auto alining while the robot is being operate
        if (Math.abs(desiredState.speedMetersPerSecond) < 0.09){
            stop();
            return;
        }

        // The actual turning encoder rotation
        var encoder_rotation = new Rotation2d(turning_encoder.getPosition());

        // Optimiza el estado de referencia para evitar giros mayores a 90 grados
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, encoder_rotation);

        // Escala la velocidad por el coseno del error angular
        state.speedMetersPerSecond *= state.angle.minus(encoder_rotation).getCos();
        
        // The actual turning_motor_position
        double turning_motor_position = turning_encoder.getPosition();

        // Assigns a speed to the drive motor
        drive_motor.set(state.speedMetersPerSecond / Constants.MAX_SPEED);

        // Calculates the necessary set speed for turning the turning motor the specified angle
        turning_motor.set(turning_PID_controller.calculate(turning_motor_position, state.angle.getRadians()));

        // Prints the swerve status
        SmartDashboard.putString("Debug", "absolute encoder angle" + absolute_encoder.getAbsolutePosition().getValue().toString());


        
    }
}
