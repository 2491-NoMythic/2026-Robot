// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LogInputs.IndexerInputsAutoLogged;

import static frc.robot.settings.Constants.IndexerConstants.*;

import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {
  TalonFX motor_1;
  TalonFX motor_2;
  IndexerInputsAutoLogged inputs;
  
  /** Creates a new Indexer. */
  public Indexer() {
    motor_1 = new TalonFX(INDEXER_MOTOR_1_ID);
    motor_2 = new TalonFX(INDEXER_MOTOR_2_ID);
    inputs = new IndexerInputsAutoLogged();
  }

/**
   * Sets motor speed using duty cycle out
   * @param speed Motor power from -1 to 1
   */
  public void set(double speed){
    motor_1.set(speed);
    motor_2.set(speed);
  }

  /**
   * Sets motor power to zero
   */
  public void stop(){
    motor_1.set(0);
    motor_2.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    inputs.motor.log(motor_1);
    inputs.motor.log(motor_2);
    Logger.processInputs("Indexer", inputs);
  }
}
