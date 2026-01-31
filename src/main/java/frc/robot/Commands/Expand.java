package frc.robot.Commands;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hopper;
import frc.robot.LogInputs.HopperInputsAutoLogged;

public class Expand extends Command{
    TalonFX hopperWideMotor;
    TalonFX hopperTallMotor;
    Hopper hopperSubsystem;
    HopperInputsAutoLogged inputs;
    
  public Expand(Hopper hopperSubsystem, HopperInputsAutoLogged inputs) {
    this.hopperSubsystem = hopperSubsystem;
    addRequirements(hopperSubsystem);
    this.inputs = inputs;
  }
    @Override
    public void initialize() {
      hopperSubsystem.setWideHopper(1);
    }

    @Override
  public void execute() {}

    @Override
  public void end(boolean interrupted) {
    hopperSubsystem.setWideHopper(0);
  }

    @Override
  public boolean isFinished(){
    return inputs.hopperExpandedInput;
  }
}


