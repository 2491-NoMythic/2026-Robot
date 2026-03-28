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
  
  public void initialize() {
    timer.reset ();
    timer.start ();
  }

  public void execute() {
    if (timer.get() > 1) {
        timer.reset();
    } 
    if (timer.get() < 0.75) {
        intake.retractIntake();
    } else {
        intake.deployIntake();
    }
  }
  public boolean isFinished() {
    return false;
  }
}
 
  
