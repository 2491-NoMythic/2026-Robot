package frc.robot.Commands;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.LogInputs.IntakeInputs;
import frc.robot.LogInputs.IntakeInputsAutoLogged;

public class Expand extends Command{
    TalonFX deployer;
    Intake intake;
    IntakeInputs inputs;
    
  public Expand(Intake intake, IntakeInputs inputs) {
    this.intake = intake;
    addRequirements(intake);
    this.inputs = inputs;
  }
    @Override
    public void initialize() {
      intake.deployIntake();;
    }

    @Override
  public void execute() {}

    @Override
  public void end(boolean interrupted) {
    intake.stopDeployer();
  }

    @Override
  public boolean isFinished(){
    return inputs.IntakeInputs;
  }
}


