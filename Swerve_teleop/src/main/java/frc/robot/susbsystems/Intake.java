// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.susbsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilidades.HardwareMap;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  public Intake() {
    
  }
  public void take(){
    HardwareMap.intake_front_wheel.set(1);
    HardwareMap.intake_back_wheel.set(1);
  }

  public void give(){
    HardwareMap.intake_front_wheel.set(-1);
    HardwareMap.intake_back_wheel.set(-1);
  }
  
  @Override
  public void periodic() {
  }
}
