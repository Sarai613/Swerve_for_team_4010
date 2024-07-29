package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.susbsystems.Intake;

public class IntakeJoystick extends Command{
    Intake intake;
    Supplier<Boolean> activate_intake, inverse_intake_activation;

    public IntakeJoystick(Intake intake,
    Supplier<Boolean> activate_intake, Supplier<Boolean> inverse_intake_activation){
        this.intake = intake;
        this.activate_intake = activate_intake;
        this.inverse_intake_activation = inverse_intake_activation;

        addRequirements(intake);
    }

    @Override
    public void execute(){
        if (activate_intake.get()) {
            intake.take();
        } else {
            intake.stop();
        }
        
        if (inverse_intake_activation.get()) {
            intake.give();
        } else {
            intake.stop();
        }
    }

    @Override
    public void end(boolean interrupted){
        intake.stop();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
