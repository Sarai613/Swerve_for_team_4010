// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.susbsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  private final CANSparkMax intake_front_wheel = new CANSparkMax(33, MotorType.kBrushless);
  private final CANSparkMax intake_back_wheel = new CANSparkMax(23, MotorType.kBrushless);
  private final SlewRateLimiter fRateLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter bRateLimiter = new SlewRateLimiter(3);

  /** Creates a new Intake. */
  public Intake() {
    
  }
  public void take(){
    intake_front_wheel.set(fRateLimiter.calculate(1));
    intake_back_wheel.set(bRateLimiter.calculate(1));
    SmartDashboard.putString("intake status", "activando el intake para agarrar");
  }

  public void stop(){
    intake_front_wheel.set(0);
    intake_back_wheel.set(0);
    SmartDashboard.putString("intake status", "deteniendo el intake");
  }

  public void give(){
    intake_front_wheel.set(fRateLimiter.calculate(-1));
    intake_back_wheel.set(bRateLimiter.calculate(-1));
    SmartDashboard.putString("intake status", "activando el intake para regresar");
  }
  
  @Override
  public void periodic() {
  }
}
