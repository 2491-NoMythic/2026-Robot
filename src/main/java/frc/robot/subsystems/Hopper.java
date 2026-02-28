// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LogInputs.HopperInputsAutoLogged;

import static frc.robot.settings.Constants.DriveConstants.CANIVORE_DRIVETRAIN;
import static frc.robot.settings.Constants.HopperConstants.*;

import org.littletonrobotics.junction.Logger;

public class Hopper extends SubsystemBase {
  TalonFX hopperRollerMotor;
  boolean hopperExpanded = false;
  HopperInputsAutoLogged inputs;

  /** Creates a new Hopper. */
  public Hopper() {
    hopperRollerMotor = new TalonFX(HOPPER_MOTOR_ID, CANIVORE_DRIVETRAIN);
    hopperRollerMotor.getConfigurator().apply(HOPPER_CONFIG);
    inputs = new HopperInputsAutoLogged();
  }

  /**
   * Sets motor speed
   * @param speed Motor power from -1 to 1
   * @param motor TalonFX 
   * 
   */
  public void setMotor(double speed, TalonFX motor){
    motor.set(speed);
  }

  public void setVelocity(double RPS){
    hopperRollerMotor.setControl(new VelocityVoltage(RPS));
  }

  public void stop() {
    setHopperRoller(0);
  }

  /**
   * feeds to indexer
   * @param speed Motor power from -1 to 1
   * 
   */
  public void setHopperRoller(double speed){
    hopperRollerMotor.set(speed);
  }

  /**
   * sets the hopper rollers to the HOPPER_ROLLER_SPEED
   */
  public void feedIndexer() {
    setVelocity(HOPPER_ROLLER_SPEED_RPS);
  }

  /**
   * checks if motor is stalled by checking if Voltage is applied but roller's aren't moving
   * @return true if the motor is stalled
   */
  public boolean isStalled() {
    return hopperRollerMotor.getMotorVoltage().getValueAsDouble() > 1 && hopperRollerMotor.getVelocity().getValueAsDouble() < 1;
  }

  /**
   * up position will be gotten by limit switch, down will be gotten by current spike
   * @return hopperExpanded (Boolean, true = expanded, false = retracted)
   */
  public boolean getHopperExpanded(){
    return hopperExpanded;
  }
 
  public void periodic() {
    inputs.hopperExpandedInput = this.getHopperExpanded();
    inputs.tallMotorInput.log(hopperRollerMotor);

    Logger.processInputs("Hopper", inputs);
}
}
