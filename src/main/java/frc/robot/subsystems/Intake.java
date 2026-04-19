// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LogInputs.IntakeInputsAutoLogged;

import static frc.robot.settings.Constants.DriveConstants.CANIVORE_DRIVETRAIN;
import static frc.robot.settings.Constants.IntakeConstants.*;

import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  TalonFXS rollerOne;
  TalonFXS rollerTwo;
  TalonFX deployer;
  TalonFXSConfiguration intakeConfig;
  IntakeInputsAutoLogged inputs;
  CANcoder absoluteEncoder;
  double targetedPosition;
  boolean targetingDeployedPosition;

  /** Creates a new Intake. */
  public Intake() {
    rollerOne = new TalonFXS(INTAKE_ROLLER_ONE_ID);
    rollerTwo = new TalonFXS(INTAKE_ROLLER_TWO_ID);
    deployer = new TalonFX(INTAKE_DEPLOYER_ID, CANIVORE_DRIVETRAIN);
    absoluteEncoder = new CANcoder(INTAKE_ENCODER_ID, CANIVORE_DRIVETRAIN);
    rollerOne.getConfigurator().apply(INTAKE_ROLLER_ONE_CONFIG);
    rollerTwo.setControl(new Follower(INTAKE_ROLLER_ONE_ID, MotorAlignmentValue.Opposed));
    deployer.getConfigurator().apply(INTAKE_DEPLOYER_CONFIG);
    inputs = new IntakeInputsAutoLogged();
  }

  /**
   * Sets intake speed using duty cycle out
   * 
   * @param speed Motor power from -1 to 1
   */
  public void setWheels(double speed) {
    rollerOne.set(speed);
  }

  public void setVelocity(double RPS){
    rollerOne.setControl(new VelocityVoltage(RPS));
  }

  /**
   * runs the intake at the INTAKE_SPEED_RPS velocity
   */
  public void feedHopper() {
    setVelocity(INTAKE_SPEED_RPS);
  }

  /**
   * Sets motor power to zero
   */
  public void stopWheels() {
    rollerOne.set(0);
  }

  /**
   * Sets velocity target for intake motor
   * 
   * @param speed RPS
   */
  public void setWheelsVelocity(double speed) {
    rollerOne.setControl(new VelocityVoltage(speed));
  }

  public void deployIntake(){
    setIntakeAngle(INTAKE_DEPLOYED_POSITION);
    targetedPosition = INTAKE_DEPLOYED_POSITION;
    targetingDeployedPosition = true;
  }

  public boolean getIsDeployed() {
    return inputs.deployerMotor.position > INTAKE_DEPLOYED_POSITION - 0.05;
  }

  public boolean getIsRetracted() {
    return inputs.deployerMotor.position < INTAKE_RETRACTED_POSITION + 0.05;
  }

  public void retractIntake(){
    setIntakeAngle(INTAKE_RETRACTED_POSITION);
    targetedPosition = INTAKE_RETRACTED_POSITION;
    targetingDeployedPosition = false;
  }

  public void stopDeployer(){
    targetingDeployedPosition = false;
    deployer.stopMotor();
  }

  public void holdPosition(){
    deployer.setControl(new VoltageOut(2));
  }

  public void setIntakeAngle(double rotations) {
    deployer.setControl(new PositionVoltage(rotations));
    targetedPosition = rotations;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    inputs.wheelsMotor.log(rollerOne);
    inputs.deployerMotor.log(deployer);
    inputs.forwardLimitSwitch = deployer.getForwardLimit().getValueAsDouble() > 0.5;
    inputs.reverseLimitSwitch = deployer.getReverseLimit().getValueAsDouble() > 0.5;
    Logger.processInputs("Intake", inputs);
    if(this.getCurrentCommand() != null) {
    } else {
      SmartDashboard.putString("IntakeCurrentCommand", "null");
    }
    if(targetingDeployedPosition && deployer.getPosition().getValueAsDouble() < INTAKE_DOWN_SOFT_LIMIT && DriverStation.isTeleop()) {
      holdPosition();
    }
  }

}
