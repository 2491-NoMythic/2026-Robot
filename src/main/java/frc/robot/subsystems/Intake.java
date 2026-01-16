// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LogInputs.IntakeInputsAutoLogged;

import static frc.robot.settings.Constants.IntakeConstants.*;

import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  TalonFX motor;
  IntakeInputsAutoLogged inputs;
  /** Creates a new Intake. */
  public Intake() {
    motor = new TalonFX(INTAKE_MOTOR_ID);
    inputs = new IntakeInputsAutoLogged();
  }

/**
   * Sets motor speed using duty cycle out
   * @param speed Motor power from -1 to 1
   */
  public void set(double speed){
    motor.set(speed);
  }

  /**
   * Sets motor power to zero
   */
  public void stop(){
    motor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    inputs.motor.log(motor);
    Logger.processInputs("Intake", inputs);
  }
}
