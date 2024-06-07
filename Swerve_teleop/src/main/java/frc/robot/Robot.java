// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.Autonomous.Trayectories.output.TrajectoryFollower;
import frc.robot.swerve.Chassis;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.utilidades.Constants;


public class Robot extends TimedRobot {
  private final XboxController controller = new XboxController(0);
  private final Chassis swerve = new Chassis();
  private final SwerveSubsystem swerve_odometry = new SwerveSubsystem();
  TrajectoryFollower trajectoryFollower = new TrajectoryFollower(swerve, swerve_odometry);
  Command command;
  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter rotLimiter = new SlewRateLimiter(3);

  StructArrayPublisher<SwerveModuleState> swerve_state_publisher = NetworkTableInstance.getDefault()
        .getStructArrayTopic("MyStates", SwerveModuleState.struct).publish();

  StructArrayPublisher<SwerveModuleState> swerve_desired_state_publisher = NetworkTableInstance.getDefault()
        .getStructArrayTopic("desiredStates", SwerveModuleState.struct).publish();


  StructPublisher<Pose2d> odometry_publisher = NetworkTableInstance.getDefault()
    .getStructTopic("MyPose", Pose2d.struct).publish();

  StructArrayPublisher<Pose2d> odometry_array_publisher = NetworkTableInstance.getDefault()
    .getStructArrayTopic("MyPoseArray", Pose2d.struct).publish();

    

  @Override
  public void robotInit(){
    swerve_odometry.zeroHeading();
  }

  @Override
  public void robotPeriodic() {
    // WPILib
        swerve_odometry.updateOdometry();
        swerve_odometry.updateModuleStates();
        swerve_state_publisher.set(swerve_odometry.swerve_module_states);
        swerve_desired_state_publisher.set(swerve.moduleStates);
        odometry_publisher.set(swerve_odometry.getPose());
  }

  @Override
  public void autonomousInit(){
    command = trajectoryFollower.getAutonomousCommand();
    command.schedule();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    CommandScheduler.getInstance().run();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    driveWithJoystick(true);
  }

  private void driveWithJoystick(boolean field_relative) {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    final var xSpeed =
        -xspeedLimiter.calculate(MathUtil.applyDeadband(controller.getLeftY(), Constants.JOYSTICK_DEADZONE))
            * Constants.MAX_SPEED;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    final var ySpeed =
        -yspeedLimiter.calculate(MathUtil.applyDeadband(controller.getLeftX(), Constants.JOYSTICK_DEADZONE))
            * Constants.MAX_SPEED;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    final var rot =
        -rotLimiter.calculate(MathUtil.applyDeadband(controller.getRightX(), Constants.JOYSTICK_DEADZONE))
            * Constants.MAX_ANGULAR_SPEED;

    swerve.drive(xSpeed, ySpeed, rot, field_relative);
  }
}
