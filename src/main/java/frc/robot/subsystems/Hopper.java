// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LogInputs.HopperInputsAutoLogged;

import static frc.robot.settings.Constants.HopperConstants.*;

import org.littletonrobotics.junction.Logger;

public class Hopper extends SubsystemBase {
  TalonFX hopperRollerMotor;
  HopperInputsAutoLogged inputs;

  /** Creates a new Hopper. */
  public Hopper() {
    hopperRollerMotor = new TalonFX(HOPPER_MOTOR_ID);
    hopperRollerMotor.getConfigurator().apply(HOPPER_CONFIG);
    inputs = new HopperInputsAutoLogged();
  }

  public void setVelocity(double RPS){
    hopperRollerMotor.setControl(new VelocityVoltage(RPS));
  }

  /**
   * feeds to indexer
   * @param speed Motor power from -1 to 1
   * 
   */
  public void setHopperRoller(double speed){
    hopperRollerMotor.set(speed);
  }

  public void stop() {
    setHopperRoller(0);
  }

  /**
   * sets the hopper rollers to the HOPPER_ROLLER_SPEED
   */
  public void feedIndexer() {
    setVelocity(HOPPER_ROLLER_SPEED_RPS);
  }

  /**
   * checks if motor is stalled by checking if Voltage is applied but rollers aren't moving
   * @return true if the motor is stalled
   */
  public boolean isStalled() {
    return hopperRollerMotor.getMotorVoltage().getValueAsDouble() > 1 && hopperRollerMotor.getVelocity().getValueAsDouble() < 1;
  }
 
  public void periodic() {
    inputs.tallMotorInput.log(hopperRollerMotor);

    Logger.processInputs("Hopper", inputs);
    if(this.getCurrentCommand() != null) {
      SmartDashboard.putString("HopperCurrentCommand", this.getCurrentCommand().toString());
    } else {
      SmartDashboard.putString("HopperCurrentCommand", "null");
    }
  }
}
