package frc.Autonomous.Trayectories.output;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.swerve.Chassis;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.utilidades.Constants;

public class TrajectoryFollower {
    private Chassis chassis;
    private TrajectoryConfig trajectory_config;
    private Trajectory trajectory;
    private PIDController x_controller;
    private PIDController y_controller;
    private ProfiledPIDController z_controller;
    private SwerveControllerCommand swerve_controller_command;
    private SwerveSubsystem swerveSubsystem;

    public TrajectoryFollower(Chassis chassis, SwerveSubsystem swerveSubsystem) {
        this.chassis = chassis;
        this.swerveSubsystem = swerveSubsystem;
        
    }

    public Command getAutonomousCommand(){
        // Initialize trajectory configurations
        trajectory_config = new TrajectoryConfig(
            Constants.AUTONOMOUS_MAX_SPEED, 
            Constants.AUTONOMOUS_MAX_ACCELERATION
        ).setKinematics(chassis.robot_kinematics);

        // Create a smooth trajectory by some given points in the field
        trajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)), // The initial position of the robot
            List.of( // A list of points that the robot should go before going to the final
                new Translation2d(1, 0), 
                new Translation2d(1, -1)
            ),
            new Pose2d(2, -1, Rotation2d.fromDegrees(180)), // The final pose of the robot
            trajectory_config // The trajectory configuration
        );

        // Define the PID controllers
        x_controller = new PIDController(Constants.AUTONOMOUS_P_X, 0, 0);
        y_controller = new PIDController(Constants.AUTONOMOUS_P_Y, 0, 0);
        z_controller = new ProfiledPIDController(
            Constants.AUTONOMOUS_P_Z, 0, 0, 
            Constants.AUTONOMOUS_Z_CONSTRAIT
        );

        // Enable continuous input for the z_controller
        z_controller.enableContinuousInput(-Math.PI, Math.PI); // This allows the z_controller to do a 360 grades turn by jumping from 180 to -180

        //Construct the command to follow the trajectory
        swerve_controller_command = new SwerveControllerCommand(
            trajectory,
            swerveSubsystem::getPose,
            chassis.robot_kinematics,
            x_controller,
            y_controller,
            z_controller,
            swerveSubsystem::setStates,
            swerveSubsystem);



            // 5. Add some init and wrap-up, and return everything
            return new SequentialCommandGroup(
                new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory.getInitialPose())),
                swerve_controller_command,
                new InstantCommand(() -> swerveSubsystem.stopModules()));
    }
}
