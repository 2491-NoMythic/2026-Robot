// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LogInputs.IntakeInputsAutoLogged;

import static frc.robot.settings.Constants.DriveConstants.CANIVORE_DRIVETRAIN;
import static frc.robot.settings.Constants.IntakeConstants.*;

import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  TalonFXS wheels;
  TalonFX deployer;
  TalonFXSConfiguration intakeConfig;
  IntakeInputsAutoLogged inputs;

  /** Creates a new Intake. */
  public Intake() {
    wheels = new TalonFXS(INTAKE_WHEELS_ID, CANIVORE_DRIVETRAIN);
    deployer = new TalonFX(INTAKE_DEPLOYER_ID, CANIVORE_DRIVETRAIN);
    wheels.getConfigurator().apply(INTAKE_WHEELS_CONFIG);
    deployer.getConfigurator().apply(INTAKE_DEPLOYER_CONFIG);
    inputs = new IntakeInputsAutoLogged();
  }

  /**
   * Sets intake speed using duty cycle out
   * 
   * @param speed Motor power from -1 to 1
   */
  public void setWheels(double speed) {
    wheels.set(speed);
  }

  /**
   * Sets motor power to zero
   */
  public void stopWheels() {
    wheels.set(0);
  }

  /**
   * Sets velocity target for intake motor
   * 
   * @param speed RPS
   */
  public void setWheelsVelocity(double speed) {
    wheels.setControl(new VelocityVoltage(speed));
  }

  public void deployIntake(){
    deployer.set(0.1);
  }

  public boolean getIsDeployed() {
    return inputs.forwardLimitSwitch;
  }

  public boolean getIsRetracted() {
    return inputs.reverseLimitSwitch;
  }

  public void retractIntake(){
    deployer.set(-0.1);
  }

  public void stopDeployer(){
    deployer.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    inputs.wheelsMotor.log(wheels);
    inputs.deployerMotor.log(deployer);
    inputs.forwardLimitSwitch = deployer.getForwardLimit().getValueAsDouble() > 0.5;
    inputs.reverseLimitSwitch = deployer.getReverseLimit().getValueAsDouble() > 0.5;
    Logger.processInputs("Intake", inputs);
  }

}
