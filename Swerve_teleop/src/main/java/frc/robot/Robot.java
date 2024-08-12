// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.susbsystems.Intake;
import frc.robot.susbsystems.Shooter;
import frc.robot.susbsystems.Swerve;


public class Robot extends TimedRobot {
  private final Swerve swerve = new Swerve();
  //private final Intake intake = new Intake();
  private final Shooter shooter = new Shooter();
  RobotContainer robotContainer = new RobotContainer(swerve, shooter);
  Command test_command = robotContainer.getTestCommand();
  Command autonomous_command = robotContainer.getAutonomousCommand();
  Translation2d[] mid_points = {
    new Translation2d(0, 0),
    new Translation2d(2, 0), 
    new Translation2d(2, -2),
    new Translation2d(4, -2)
  };

  StructArrayPublisher<SwerveModuleState> swerve_state_publisher = NetworkTableInstance.getDefault()
        .getStructArrayTopic("MyStates", SwerveModuleState.struct).publish();



  StructPublisher<Pose2d> odometry_publisher = NetworkTableInstance.getDefault()
    .getStructTopic("MyPose", Pose2d.struct).publish();

  StructPublisher<Pose2d> odometry_end_publisher = NetworkTableInstance.getDefault()
    .getStructTopic("myEnd", Pose2d.struct).publish();

  StructArrayPublisher<Translation2d> mid_points_publisher = NetworkTableInstance.getDefault()
    .getStructArrayTopic("MyMidPoseArray", Translation2d.struct).publish();

    

  @Override
  public void robotInit(){
    swerve.zeroHeading();
  }

  @Override
  public void robotPeriodic() {
    // WPILib
        swerve.updateOdometry();
        swerve.updateModuleStates();
        swerve_state_publisher.set(swerve.swerve_module_states);
        odometry_publisher.set(swerve.getPose());
        odometry_end_publisher.set(new Pose2d(4, -2, Rotation2d.fromDegrees(180)));
        mid_points_publisher.set(mid_points);

    CommandScheduler.getInstance().run(); // Runs the scheduled commands
  }

  @Override
  public void autonomousInit(){

    if (autonomous_command != null){
      autonomous_command.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

  }

  @Override
  public void teleopInit(){
    if (autonomous_command != null){
      autonomous_command.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit(){
    CommandScheduler.getInstance().cancelAll();
    test_command.schedule();
  }

}
