// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.susbsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  public static final CANSparkMax intake_front_wheel = new CANSparkMax(24, MotorType.kBrushless);
  public static final CANSparkMax intake_back_wheel = new CANSparkMax(23, MotorType.kBrushless);

  /** Creates a new Intake. */
  public Intake() {
    
  }
  public void take(){
    intake_front_wheel.set(.5);
    intake_back_wheel.set(.5);
  }

  public void stop(){
    intake_front_wheel.set(0);
    intake_back_wheel.set(0);
  }

  public void give(){
    intake_front_wheel.set(-.5);
    intake_back_wheel.set(-.5);
  }
  
  @Override
  public void periodic() {
  }
}
