// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LogInputs.HopperInputsAutoLogged;
import static frc.robot.settings.Constants.HopperConstants.*;

import org.littletonrobotics.junction.Logger;

public class Hopper extends SubsystemBase {
  TalonFX hopperMotor;
  TalonFX hopperMotorTwo;
  boolean expandDesired = false; 
  boolean hopperExpanded = false;
  HopperInputsAutoLogged inputs;

  /** Creates a new Hopper. */
  public Hopper() {
    hopperMotor = new TalonFX(HOPPER_MOTOR_ID);
    hopperMotor = new TalonFX(HOPPER_MOTOR_TWO_ID);
    inputs = new HopperInputsAutoLogged();
  }

  /**
   * Sets motor speed using duty cycle out
   * @param speed Motor power from -1 to 1
   */
  private void setMotor(double speed){
    hopperMotor.set(speed);
    hopperMotorTwo.set(speed);
  }

  /**
   * runs the hopper motor up
   */
  public void expandHopper() {
    setMotor(1);
  }
  /**
   * runs the hopper motor down, likely will almost never be used
   */
  public void retractHopper() { 
    setMotor(-1);
  }

  /**
   * Sets motor power to zero
   */
  public void stop(){
    setMotor(0);
  }

  public boolean getHopperPosition(){
    //up position will be gotten by limit switch, down will be gotten by current spike
    return hopperExpanded;
  }
  public boolean getDesire(){
    return expandDesired;
  }

  public void periodic() {
    inputs.hopperExpandedInput = this.getHopperPosition();
    inputs.expandDesiredInput = this.getDesire();
    inputs.motorInput.log(hopperMotor);
    inputs.motorTwoInput.log(hopperMotorTwo);

    Logger.processInputs("Hopper", inputs);
    //other periodic code
}
}
