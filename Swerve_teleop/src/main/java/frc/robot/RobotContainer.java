package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.autonomous.TrajectoryFollower;
import frc.robot.commands.SwerveDriveJoystick;
import frc.robot.commands.Test;
import frc.robot.susbsystems.Intake;
import frc.robot.susbsystems.Shooter;
import frc.robot.susbsystems.Swerve;
import frc.robot.utilidades.Constants;

public class RobotContainer {
    private final Joystick driverJoystick = new Joystick(Constants.DRIVER_PORT);
    private final Joystick coDriverJoystick = new Joystick(Constants.CODRIVER_PORT);
    private final TrajectoryFollower trajectoryFollower;
    
    public RobotContainer(Swerve swerve, Intake intake, Shooter shooter){
        swerve.setDefaultCommand(new SwerveDriveJoystick(
            swerve,
            () -> driverJoystick.getRawAxis(Constants.DRIVER_X_AXIS),
            () -> -driverJoystick.getRawAxis(Constants.DRIVER_Y_AXIS),
            () -> driverJoystick.getRawAxis(Constants.DRIVER_Z_AXIS),
            () -> !driverJoystick.getRawButton(Constants.DRIVER_ROBOT_ORIENTED_BUTTON)));

            

        trajectoryFollower = new TrajectoryFollower(swerve);

        configureButtonBindings();
    }

    private void configureButtonBindings(){
        //new JoystickButton(driverJoystick, 2).whenPressed(() -> swerve.zeroHeading());
    }

    public Command getAutonomousCommand(){
        return trajectoryFollower.getAutonomousCommand();
    }

    public Command getTestCommand(){
        return new Test();
    }
}
