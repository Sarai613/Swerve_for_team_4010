// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Utils.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class SwerveModule {
  private static final double kWheelRadius = Constants.WHEEL_RADIUS;
  private static final int kEncoderResolution = Constants.ENCODER_RESOLUTION;

  private static final double kModuleMaxAngularVelocity = Constants.MAX_ANGULAR_VELOCITY;
  private static final double kModuleMaxAngularAcceleration = Constants.MAX_ANGULAR_ACCELERATION; 

  private final CANSparkMax m_driveMotor;
  private final CANSparkMax m_turningMotor;


  // Gains are for example purposes only - must be determined for your own robot!
  private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(1, 3);
  private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(1, 0.5);

  public SwerveModule(int drive_spark_id,int turning_spark_id) {
    
    this.m_driveMotor = new CANSparkMax(drive_spark_id, MotorType.kBrushless);
    this.m_turningMotor = new CANSparkMax(turning_spark_id, MotorType.kBrushless);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {

    double driveSpeed = m_driveMotor.getEncoder().getVelocity();

    double turningPosition = m_turningMotor.getEncoder().getPosition();

    SwerveModuleState state = new SwerveModuleState(driveSpeed, new Rotation2d(turningPosition));

    return state;
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        m_driveMotor.getEncoder().getPosition(), new Rotation2d(m_turningEncoder.getEncoder().getPosition()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    var encoderRotation = new Rotation2d(m_turningEncoder.getEncoder().getPosition());

    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, encoderRotation);

    // Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
    // direction of travel that can occur when modules change directions. This results in smoother
    // driving.
    state.speedMetersPerSecond *= state.angle.minus(encoderRotation).getCos();

    drive_output = desired_speed
  }
}
