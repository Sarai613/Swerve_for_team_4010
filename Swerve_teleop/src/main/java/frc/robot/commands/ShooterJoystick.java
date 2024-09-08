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
            shooter.setReloader(1);;
        } else if (activate_shooter_flying_wheel.get()){
            shooter.setLauncher(1);;
        } else if  (activate_shooter.get()){
            shooter.setReloader(1);
            shooter.setLauncher(1);
        } else {
            shooter.setReloader(0);
            shooter.setLauncher(0);
        }

        double real_rotator_power = rotator_power.get();
        real_rotator_power = Math.abs(real_rotator_power) > Constants.JOYSTICK_DEADZONE ? real_rotator_power : 0.0;

        shooter.setRotatorPower(real_rotator_power);
    }

    @Override
    public void end(boolean interrupted){
        shooter.setReloader(0);
        shooter.setLauncher(0);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
