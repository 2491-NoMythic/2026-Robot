// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LogInputs.IntakeInputsAutoLogged;

import static frc.robot.settings.Constants.IntakeConstants.*;

import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  TalonFX leftIntakeMotor;
  TalonFX rightIntakeMotor;
  IntakeInputsAutoLogged inputs;

  /** Creates a new Intake. */
  public Intake() {
    leftIntakeMotor = new TalonFX(LEFT_INTAKE_MOTOR_ID);
    rightIntakeMotor = new TalonFX(RIGHT_INTAKE_MOTOR_ID);
    leftIntakeMotor.getConfigurator().apply(INTAKE_CONFIG);
    rightIntakeMotor.getConfigurator().apply(INTAKE_CONFIG);
    rightIntakeMotor.setControl(new Follower(LEFT_INTAKE_MOTOR_ID, MotorAlignmentValue.Aligned));
  }

  /**
   * Sets motor speed using duty cycle out
   * 
   * @param speed Motor power from -1 to 1
   */
  public void set(double speed) {
    leftIntakeMotor.set(speed);
    rightIntakeMotor.set(speed);
  }

  /**
   * Sets motor power to zero
   */
  public void stop() {
    leftIntakeMotor.set(0);
    rightIntakeMotor.set(0);
  }

  /**
   * Sets velocity target for intake motor
   * 
   * @param speed RPS
   */
  public void setVelocity(double speed) {
    leftIntakeMotor.setControl(new VelocityVoltage(speed));
    rightIntakeMotor.setControl(new VelocityVoltage(speed));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    inputs.leftMotor.log(leftIntakeMotor);
    inputs.rightMotor.log(rightIntakeMotor);
    Logger.processInputs("Intake", inputs);
  }
}
