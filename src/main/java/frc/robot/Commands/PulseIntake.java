package frc.robot.Commands;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.LogInputs.IntakeInputs;
import frc.robot.LogInputs.IntakeInputsAutoLogged;

public class PulseIntake extends Command {
    TalonFX deployer;
    Intake intake;
    Timer timer;


 public PulseIntake(Intake intake) {
    this.intake = intake;
    timer = new Timer();
    addRequirements(intake);
  }
  
  @Override
  public void initialize() {
    timer.reset ();
    timer.start ();
  }

  @Override
  public void execute() {
    intake.feedHopper();
    if (timer.get() > 1) {
        timer.reset();
    } 
    if (timer.get() < 0.75) {
        intake.setIntakeAngle(-0.28);
    } else {
        intake.deployIntake();
    }
  }

  @Override
  public void end(boolean interrupted) {
    intake.deployIntake();
    intake.stopWheels();
    
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
 
  
