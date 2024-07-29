// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you c3an modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.susbsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilidades.Constants;

public class Shooter extends SubsystemBase {

  public final TalonSRX shooter_launcher = new TalonSRX(4);
  public final TalonSRX shooter_reloader = new TalonSRX(1);
  public final TalonSRX shooter_spin_motor = new TalonSRX(2);
  public final TalonSRX shooter_spin_motor_2 = new TalonSRX(3);
  private final SlewRateLimiter lRateLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter rRateLimiter = new SlewRateLimiter(3);


  /** Creates a new Shooter. */
  public Shooter() {
    shooter_spin_motor.set(TalonSRXControlMode.Position, 0);;
  }

  PhoenixPIDController rotatorPIDController = new PhoenixPIDController(1, 0.2, 15);

  public void chargeLauncher(){
    shooter_launcher.set(TalonSRXControlMode.PercentOutput, lRateLimiter.calculate(1));
  }

  public void reload(){
      shooter_launcher.set(TalonSRXControlMode.PercentOutput, rRateLimiter.calculate(.8));
  }

  public void moveBoth(){
    chargeLauncher();
    reload();
  }

  public void stopLauncher(){
    shooter_launcher.set(TalonSRXControlMode.PercentOutput, 0);
  }
  
  public void stopReloader(){
    shooter_launcher.set(TalonSRXControlMode.PercentOutput, 0);
  }
  
  public void stopRotator(){
    setRotatorPower(0);
  }

  public void stopRollers(){
    stopLauncher();
    stopReloader();
  }

  public void stop(){
    stopRollers();
    stopRotator();

  }

  public double[] getRotatorPosition(){
    return new double[] {shooter_spin_motor.getSelectedSensorPosition(), shooter_spin_motor_2.getSelectedSensorPosition()};
  }

  public Rotation2d[] getRotatorRotation(){
    return new Rotation2d[] {
      new Rotation2d(Constants.TICKS_PER_RADIAN_OF_THE_ROTATOR * getRotatorPosition()[0]),
      new Rotation2d(Constants.TICKS_PER_RADIAN_OF_THE_ROTATOR * getRotatorPosition()[1])};
  }

  public void setAngle(Rotation2d angle){ 
    shooter_spin_motor.set(TalonSRXControlMode.PercentOutput, rotatorPIDController.calculate(getRotatorPosition()[0], angle.getRadians() * Constants.TICKS_PER_RADIAN_OF_THE_ROTATOR, System.currentTimeMillis()) * .2);
    shooter_spin_motor_2.set(TalonSRXControlMode.PercentOutput, rotatorPIDController.calculate(getRotatorPosition()[1], angle.getRadians() * Constants.TICKS_PER_RADIAN_OF_THE_ROTATOR, System.currentTimeMillis()) * .2);
  }

  public void setRotatorPower(double power){
    shooter_spin_motor.set(TalonSRXControlMode.PercentOutput, power);
    shooter_spin_motor_2.set(TalonSRXControlMode.PercentOutput, power);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
