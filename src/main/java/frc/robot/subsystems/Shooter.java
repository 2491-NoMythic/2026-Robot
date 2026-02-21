// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.LogInputs.ShooterInputsAutoLogged;
import frc.robot.helpers.MythicalMath;

import static frc.robot.settings.Constants.ShooterConstants.*;

import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  TalonFX shootMotor;
  Servo leftHoodActuator;
  Servo rightHoodActuator;
  ShooterInputsAutoLogged inputs;
  double desiredPosition;
  boolean positionControlOn;
  /** Creates a new Shooter. */
  public Shooter() {
    shootMotor = new TalonFX(SHOOTER_MOTOR_ID);
    shootMotor.getConfigurator().apply(SHOOTER_CONFIG);
    leftHoodActuator = new Servo(HOOD_LEFT_ACTUATOR_ID);
    rightHoodActuator = new Servo(HOOD_RIGHT_ACTUATOR_ID);
    inputs = new ShooterInputsAutoLogged();
    //SmartDashboard.putNumber("hoodPosition", 0);
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
   * @param angle angle to set the hood to, in radians
   */
  public void setHoodAngle(double angleInRadians){
    desiredPosition = InchPositionToActuatorConstrainedPercent(                                                                   //54.328 degrees
      MythicalMath.ServoExtensionToReachHoodAngleInRadians(angleInRadians, 6.610, 8.134, 4.914, 0.948202476)
    );
    
    positionControlOn = true;
  }
  public void setHoodAngleUp(){
    desiredPosition = 0.8;
    positionControlOn = true;
  }

  public void setHoodAngleDown(){
    desiredPosition = 0.2;
    positionControlOn = true;
  }

  public double InchPositionToActuatorConstrainedPercent(double inches){
    return (inches / 3.93701) * 0.6 + 0.2; //Actuator "prefers" (demands) values from 0.2 to 0.8
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    inputs.shootMotor.log(shootMotor);
    Logger.processInputs("Shooter", inputs);

    //logic below checks if robot is in one of four squares around the trenches
    double x = RobotState.getInstance().robotPosition.getX();
    double y = RobotState.getInstance().robotPosition.getY();
    boolean inBadxZone = false;
    boolean inBadyZone = false;
    if((3.5 < x && x < 5.5) || (11 < x && x < 13)) {
      inBadxZone = true;
    }
    if((0 < y && y < 1.75) || (6.5 < y && y < 8)) {
      inBadyZone = true;
    }
    //if we are in position control, and in one of those squares around trenches, hood all the way down
    if(positionControlOn) {
      if(inBadxZone && inBadyZone) {
        shootMotor.setControl(new PositionVoltage(HOOD_DOWN_POSITION));
      } else {
        shootMotor.setControl(new PositionVoltage(desiredPosition));
      }
    }

    //double smartActuatorValue = SmartDashboard.getNumber("hoodPosition", 0);
    //leftHoodActuator.set(smartActuatorValue);
    //System.out.print("Actuator should be at " + smartActuatorValue);
  }
}
