package frc.robot.Commands;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.LogInputs.IntakeInputs;
import frc.robot.LogInputs.IntakeInputsAutoLogged;

public class Expand extends Command{
    TalonFX deployer;
    Intake intake;
    Timer timer;
    
  public Expand(Intake intake) {
    this.intake = intake;
    timer = new Timer();
    addRequirements(intake);
  }
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    intake.deployIntake();
  }

  @Override
  public void execute() {
    if(timer.get() > 0.5) {
      intake.feedHopper();
    }
  }

    @Override
  public void end(boolean interrupted) {
    intake.stopDeployer();
    intake.feedHopper();
  }

    @Override
  public boolean isFinished(){
    return intake.getIsDeployed();
  }
}


