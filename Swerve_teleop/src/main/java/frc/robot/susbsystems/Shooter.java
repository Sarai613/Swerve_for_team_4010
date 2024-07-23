// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you c3an modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.susbsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilidades.HardwareMap;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  public Shooter() {}

  public void launch(){
    HardwareMap.shooter_launcher.set(1);
  }

  public void reload(){

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
