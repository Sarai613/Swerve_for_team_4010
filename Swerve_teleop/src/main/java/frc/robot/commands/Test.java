package frc.robot.commands;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.susbsystems.Intake;
import frc.robot.susbsystems.Shooter;
import frc.robot.susbsystems.TejuinoBoard;

public class Test extends Command{

    Intake intake;
    Shooter shooter;
    TejuinoBoard tejuinoBoard = new TejuinoBoard(0);
    ScheduledExecutorService scheduler = Executors.newScheduledThreadPool(1);
    boolean finished = false;

    public Test(Intake intake, Shooter shooter){
        this.intake = intake;
        this.shooter = shooter;
    }

    @Override
    public void initialize() {
        SmartDashboard.putString("Debug", "Initializing Test command");
    }

    @Override
    public void execute(){
        shooter.setRotatorPower(.1);
        scheduler.schedule(() -> {
                // Code to run after the delay
                shooter.setRotatorPower(-.1);
            }, 2, TimeUnit.SECONDS);
    }

    @Override
    public boolean isFinished() {
        return finished; // Adjust this condition based on when you want the command to finish
    }
}
