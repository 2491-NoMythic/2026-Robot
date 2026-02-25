// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LogInputs.IntakeInputsAutoLogged;

import static frc.robot.settings.Constants.IntakeConstants.*;

import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  TalonFX wheels;
  TalonFX deployer;
  IntakeInputsAutoLogged inputs;

  /** Creates a new Intake. */
  public Intake() {
    wheels = new TalonFX(INTAKE_WHEELS_ID);
    deployer = new TalonFX(INTAKE_DEPLOYER_ID);
    wheels.getConfigurator().apply(INTAKE_CONFIG);
    deployer.getConfigurator().apply(INTAKE_CONFIG);
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
    Logger.processInputs("Intake", inputs);
  }

}
