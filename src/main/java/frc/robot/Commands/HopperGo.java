package frc.robot.Commands;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hopper;
import frc.robot.LogInputs.HopperInputsAutoLogged;

public class HopperGo extends Command{
    TalonFX hopperMotor;
    Hopper hopperSubsystem;
    boolean hopperPosition;
    HopperInputsAutoLogged inputs;
    
  public HopperGo(Hopper hopperSubsystem, HopperInputsAutoLogged inputs) {
    this.hopperSubsystem = hopperSubsystem;
    addRequirements(hopperSubsystem);
    this.inputs = inputs;
  }
    @Override
    public void initialize() {}

    @Override
  public void execute() {
    if(inputs.hopperExpandedInput == false &&  inputs.expandDesiredInput == true) {
      hopperSubsystem.expandHopper();
    }
    else if (inputs.hopperExpandedInput == true && inputs.expandDesiredInput == false){
      hopperSubsystem.retractHopper();
    }
    else {
      hopperSubsystem.stop();
    }
  }
}


