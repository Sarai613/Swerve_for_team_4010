package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.susbsystems.Swerve;
import frc.robot.utilidades.Constants;

public class SwerveDriveJoystick extends Command{
    private final Swerve swerve;
    private final Supplier<Double> x, y, z;
    private final Supplier<Boolean> field_relative;
    private final SlewRateLimiter xLimiter, yLimiter, zLimiter;

    public SwerveDriveJoystick(Swerve swerve, 
    Supplier<Double> x, Supplier<Double> y, Supplier<Double> z,
    Supplier<Boolean> field_relative){
        this.swerve = swerve;
        this.x = x;
        this.y = y;
        this.z = z;
        this.field_relative = field_relative;
        this.xLimiter = new SlewRateLimiter(Constants.MAX_ACCELERATION_METERS_PER_SECOND);
        this.yLimiter = new SlewRateLimiter(Constants.MAX_ACCELERATION_METERS_PER_SECOND);
        this.zLimiter = new SlewRateLimiter(Constants.MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND);
        addRequirements(swerve);
    }

    @Override
    public void initialize() {

    }


    @Override
    public void execute() {
        double xSpeed = x.get();
        double ySpeed = y.get();
        double zSpeed = z.get();

        xSpeed = Math.abs(xSpeed) > Constants.JOYSTICK_DEADZONE ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > Constants.JOYSTICK_DEADZONE ? ySpeed : 0.0;
        zSpeed = Math.abs(zSpeed) > Constants.JOYSTICK_DEADZONE ? zSpeed : 0.0;

        xSpeed = xLimiter.calculate(xSpeed) * Constants.MAX_SPEED;
        ySpeed = yLimiter.calculate(ySpeed) * Constants.MAX_SPEED;
        zSpeed = zLimiter.calculate(zSpeed) * Constants.MAX_ANGULAR_SPEED;

        ChassisSpeeds chassisSpeeds;
        if (field_relative.get()){
            //Relative to field
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, zSpeed, swerve.getRotation2d());
        } else {
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, zSpeed);
        }

        SwerveModuleState[] moduleStates = swerve.robot_kinematics.toSwerveModuleStates(chassisSpeeds);

        swerve.setStates(moduleStates);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
