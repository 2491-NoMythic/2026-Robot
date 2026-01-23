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
  TalonFX wideMotor;
  TalonFX tallMotor;
  boolean wideDesired = false;
  boolean tallDesired = false;  
  boolean hopperWide = false;
  boolean hopperTall = false;
  HopperInputsAutoLogged inputs;

  /** Creates a new Hopper. */
  public Hopper() {
    wideMotor = new TalonFX(HOPPER_MOTOR_ID);
    tallMotor = new TalonFX(HOPPER_MOTOR_TWO_ID);
    inputs = new HopperInputsAutoLogged();
  }

  /**
   * Sets motor speed using duty cycle out
   * @param speed Motor power from -1 to 1
   */
  public void setMotor(double speed, TalonFX motor){
    motor.set(speed);
  }


  public boolean getHopperWide(){
    //up position will be gotten by limit switch, down will be gotten by current spike
    return hopperWide;
  }

  public boolean getHopperTall(){
    //up position will be gotten by limit switch, down will be gotten by current spike
    return hopperTall;
  }

  public boolean getWideDesire(){
    return wideDesired;
  }

  public boolean getTallDesire(){
    return wideDesired;
  }

  public void periodic() {
    inputs.hopperWideInput = this.getHopperWide();
    inputs.wideDesiredInput = this.getWideDesire();
    inputs.hopperTallInput = this.getHopperTall();
    inputs.tallDesiredInput = this.getTallDesire();
    inputs.wideMotorInput.log(wideMotor);
    inputs.tallMotorInput.log(tallMotor);

    Logger.processInputs("Hopper", inputs);
    //other periodic code
}
}
