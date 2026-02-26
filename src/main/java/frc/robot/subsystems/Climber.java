// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LogInputs.ClimberInputsAutoLogged;
import frc.robot.settings.ClimberState;

import static frc.robot.settings.Constants.ClimberConstants.*;
import static frc.robot.settings.Constants.DriveConstants.CANIVORE_DRIVETRAIN;

import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
  TalonFX motor;
  DigitalInput hallEffect;
  ClimberInputsAutoLogged inputs;
  double maxPosition;
  double desiredSpeed;
  ClimberState climberState;
  /** Creates a new Climber. */
  public Climber() {
    motor = new TalonFX(CLIMBER_MOTOR_ID, CANIVORE_DRIVETRAIN);
    motor.getConfigurator().apply(CLIMBER_CONFIG);
    inputs = new ClimberInputsAutoLogged();
    motor.setPosition(0);
    hallEffect = new DigitalInput(HALL_EFFECT_ID);
  }


  /**
   * raises the climber arm, lowering the robot unless the climber has reached the soft limit, which is relative to the climbers start position when code was deployed
   */
  public void climberUp() {
    desiredSpeed = -0.5;
  }
  /**
   *Lowers the climber arm, lifting the robot.
   */
  public void climberDown() {
    desiredSpeed = 0.5;
  }

  public boolean getHallEffect() {
    return inputs.hallEffect;
  }

  /**
   * Stops the climber
   */
  public void stop(){
    desiredSpeed = 0;
  }

  /**
   * tells what the climber is doing/what position it's in
   * @return climber position
   */
  public ClimberState getClimberState(){
    return climberState;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    inputs.motor.log(motor);
    inputs.hallEffect = !hallEffect.get();
    Logger.processInputs("Climber", inputs);

    if (inputs.hallEffect) {
      motor.setPosition(HALL_EFFECT_HEIGHT);
    }

    /**
    *When the climber goes above max height, stops it and sets the Climber Postion to Up.
    *Sets the Climber Position to Lowering/Raising when it is doing the respective action, then when we want the climber to be stopped, sets the Climber Position to Down.
    */
    if(desiredSpeed > 0 && inputs.motor.position > CLIMBER_MAX_POSITION) {
      motor.stopMotor();
      climberState = ClimberState.Up;
    } else if(desiredSpeed < 0 && climberState == ClimberState.Down) {
      motor.stopMotor();
    } else if(inputs.motor.current > 40){
      climberState = ClimberState.Down;
      motor.stopMotor();
    } else {
      motor.set(desiredSpeed);
      if(desiredSpeed < 0){
        climberState = ClimberState.LoweringClimber;
      }
      else if(desiredSpeed > 0){
        climberState = ClimberState.RaisingClimber;
      }
      else{
        climberState = ClimberState.Stopped;
      }
    }
    RobotState.getInstance().climberState = climberState;
  }
}
