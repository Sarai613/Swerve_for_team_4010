package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.susbsystems.Shooter;
import frc.robot.susbsystems.Swerve;
import frc.robot.susbsystems.TejuinoBoard;
import frc.robot.utilidades.Constants;

public class Aligner extends Command{
    private double x_distance;
    private double y_distance;
    private double distance;
    private Rotation2d yaw;
    private Rotation2d desired_angle;
    private Pose2d position;
    private Swerve swerve;
    private TejuinoBoard tejuinoBoard;
    private Shooter shooter;
    private PIDController pIDController = new PIDController(Constants.AUTONOMOUS_P_Z, Constants.AUTONOMOUS_I_Z, Constants.AUTONOMOUS_D_Z);

    public Aligner(Swerve swerve, TejuinoBoard tejuinoBoard, Shooter shooter){
        this.swerve = swerve;
        this.tejuinoBoard = tejuinoBoard;
        this.shooter = shooter;
    }

    @Override
    public void execute() {
        if (distance <= Constants.MAX_DISTANCE){
            position = swerve.getPose();
            x_distance = position.getX();
            y_distance = position.getY();
            yaw = position.getRotation();
            distance = Math.sqrt(Math.pow(x_distance, 2) + Math.pow(y_distance, 2));
            desired_angle = new Rotation2d(Math.atan2(y_distance, x_distance));
            yaw = position.getRotation();
            adjustYaw();
            adjustShooterAngle();
        } else {
            tejuinoBoard.all_leds_red(1);
            
        }
    }

    private void adjustYaw(){
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, pIDController.calculate(desired_angle.getRadians(), yaw.getRadians()), yaw);
        SwerveModuleState[] module_states = swerve.robot_kinematics.toSwerveModuleStates(speeds);
        swerve.setStates(module_states);
    }

    private void adjustShooterAngle(){
        Rotation2d angle = new Rotation2d(Math.atan(4 * Constants.SPEAKER_HEIGHT / distance));
        double velocity = Math.sqrt(19.62 * Constants.SPEAKER_HEIGHT / Math.pow(Math.sin(angle.getRadians()), 2));
        
        shooter.setAngle(angle);
        shooter.setLauncherVelocity(velocity);

        if (Math.abs(shooter.getLauncherVelocity() - velocity) >= 0.06){
            tejuinoBoard.all_leds_green(1);
        } else {
            tejuinoBoard.all_leds_yellow(1);
        }
    }
}
