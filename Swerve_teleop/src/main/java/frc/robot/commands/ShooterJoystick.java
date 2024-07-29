package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.susbsystems.Shooter;
import frc.robot.utilidades.Constants;

public class ShooterJoystick extends Command{
    Shooter shooter;
    Supplier<Boolean> activate_shooter_reloader, activate_shooter_flying_wheel, activate_shooter;
    Supplier<Double> rotator_power;
    SlewRateLimiter rotatorRateLimiter = new SlewRateLimiter(3);

    public ShooterJoystick(Shooter shooter, 
    Supplier<Boolean> activate_shooter_reloader, Supplier<Boolean> activate_shooter_flying_wheel, 
    Supplier<Boolean> activate_shooter, Supplier<Double> rotator_power){
        this.shooter = shooter;
        this.activate_shooter_reloader = activate_shooter_reloader;
        this.activate_shooter_flying_wheel = activate_shooter_flying_wheel;
        this.activate_shooter = activate_shooter;
        this.rotator_power = rotator_power;
        addRequirements(shooter);
    }


    @Override
    public void execute(){
        if (activate_shooter_reloader.get()){
            shooter.reload();
        } else {
            shooter.stopRollers();
        }

        if (activate_shooter_flying_wheel.get()){
            shooter.chargeLauncher();
        } else {
            shooter.stopRollers();
        }

        if  (activate_shooter.get()){
            if (activate_shooter_reloader.get() && !activate_shooter_flying_wheel.get()){
                shooter.chargeLauncher();
            }

            if (activate_shooter_flying_wheel.get() && !activate_shooter_reloader.get()){
                shooter.reload();
            }
        }

        double real_rotator_power = rotator_power.get();
        real_rotator_power = Math.abs(real_rotator_power) > Constants.JOYSTICK_DEADZONE ? real_rotator_power : 0.0;

        shooter.setRotatorPower(rotatorRateLimiter.calculate(real_rotator_power));
    }

    @Override
    public void end(boolean interrupted){
        shooter.stop();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
