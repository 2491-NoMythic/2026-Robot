package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class TurnOffIntake extends Command {
    Intake intake;


    public TurnOffIntake (Intake intake){
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {}

    @Override 
    public void execute() {
        intake.stopWheels();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}