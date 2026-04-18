// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.LogInputs.ShooterInputsAutoLogged;
import frc.robot.helpers.MythicalMath;

import static frc.robot.settings.Constants.DriveConstants.CANIVORE_DRIVETRAIN;
import static frc.robot.settings.Constants.ShooterConstants.*;

import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  TalonFX shootMotor1;
  TalonFX shootMotor2;
  TalonFXS hoodMotor;
  ShooterInputsAutoLogged inputs;
  double desiredPosition;
  public boolean isOn;
  /** Creates a new Shooter. */
  public Shooter() {
    shootMotor1 = new TalonFX(SHOOTER_LEFT_MOTOR_ID, CANIVORE_DRIVETRAIN);
    shootMotor1.getConfigurator().apply(SHOOTER_CONFIG);
    shootMotor2 = new TalonFX(SHOOTER_RIGHT_MOTOR_ID, CANIVORE_DRIVETRAIN);
    shootMotor2.setControl(new Follower(SHOOTER_LEFT_MOTOR_ID, MotorAlignmentValue.Opposed));
    shootMotor2.getConfigurator().apply(SHOOTER_CONFIG);
    hoodMotor = new TalonFXS(HOOD_MOTOR_ID, CANIVORE_DRIVETRAIN);
    hoodMotor.getConfigurator().apply(HOOD_MOTOR_CONFIG);
    inputs = new ShooterInputsAutoLogged();
    SmartDashboard.putNumber("PASS-TEST/shooterAngle", 30);
    SmartDashboard.putNumber("PASS-TEST/shooterSpeed", 30);
    //SmartDashboard.putNumber("hoodPosition", 0);
  }

  /**
   * Sets motor speed using duty cycle out
   * @param speed Motor power from -1 to 1
   */
  public void set(double speed){
    shootMotor1.set(speed);
  }

  /**
   * Sets motor power to zero
   */
  public void stop(){
    //shootMotor1.set(0);
    isOn = false;
  }

  /**
   * Sets velocity target for shoot motor
   * @param speed RPS
   */
  public void setVelocity(double speed){
    shootMotor1.setControl(new VelocityVoltage(speed));
  }

  public boolean isAtSpeed() {
    return shootMotor1.getVelocity().getValueAsDouble() > 22;
  }

  public void shooterOn() {
    //setVelocity(SHOOTING_SPEED_RPS);
    isOn = true;
  }

  public void setShooterToPassState() {
    setVelocity(65);
    setDesiredHoodAngle(0);
  }
  /**
   * sends a positionVoltage request to the hood motor
   * @param angle angle to set the hood to, in radians
   */
  public void setDesiredHoodAngle(double angle){
    desiredPosition = angle;
    if (desiredPosition < HOOD_DOWN_POSITION){
      desiredPosition = HOOD_DOWN_POSITION;
    }
    if (desiredPosition > HOOD_UP_POSITION){
      desiredPosition = HOOD_UP_POSITION;
    }
    hoodMotor.setControl(new PositionVoltage(desiredPosition));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    inputs.shootMotorLead.log(shootMotor1);
    inputs.shootMotorFollow.log(shootMotor2);
    inputs.hoodMotor.log(hoodMotor);
    Logger.processInputs("Shooter", inputs);
    SmartDashboard.putBoolean("SHOOTER/isAtSpeed", isAtSpeed());
    if(this.getCurrentCommand() != null) {
      SmartDashboard.putString("ShooterCurrentCommand", this.getCurrentCommand().toString());
    } else {
      SmartDashboard.putString("ShooterCurrentCommand", "null");
    }

    if(isOn) {
      setVelocity(RobotState.getInstance().desiredShooterSpeed);
    } else {
      setVelocity(0);
    }
  }
}
