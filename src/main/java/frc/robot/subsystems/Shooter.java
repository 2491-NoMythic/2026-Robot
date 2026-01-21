// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LogInputs.ShooterInputs;
import frc.robot.LogInputs.ShooterInputsAutoLogged;

import static frc.robot.settings.Constants.ShooterConstants.*;

import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  TalonFX shootMotor;
  TalonFX hoodMotor;
  ShooterInputsAutoLogged inputs;
  /** Creates a new Shooter. */
  public Shooter() {
    shootMotor = new TalonFX(SHOOTER_MOTOR_ID);
    shootMotor.getConfigurator().apply(SHOOTER_CONFIG);
    hoodMotor = new TalonFX(HOOD_MOTOR_ID);
    inputs = new ShooterInputsAutoLogged();
  }

  /**
   * Sets motor speed using duty cycle out
   * @param speed Motor power from -1 to 1
   */
  public void set(double speed){
    shootMotor.set(speed);
  }

  /**
   * Sets motor power to zero
   */
  public void stop(){
    shootMotor.set(0);
  }

  /**
   * Sets velocity target for shoot motor
   * @param speed RPS
   */
  public void setVelocity(double speed){
    shootMotor.setControl(new VelocityVoltage(speed));
  }

  /**
   * sends a positionVoltage request to the hood motor
   * @param rotations rotations to set the hood to
   */
  public void setHoodAngle(double rotations){
    hoodMotor.setControl(new PositionVoltage(rotations));
  }

  public void setHoodMotor(double speed){
    hoodMotor.set(speed);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    inputs.shootMotor.log(shootMotor);
    inputs.hoodMotor.log(hoodMotor);
    Logger.processInputs("Shooter", inputs);
  }
}
