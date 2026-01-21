// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LogInputs.ClimberInputsAutoLogged;

import static frc.robot.settings.Constants.ClimberConstants.*;

import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
  TalonFX motor;
  ClimberInputsAutoLogged inputs;
  double maxPosition;
  double desiredSpeed;
  /** Creates a new Climber. */
  public Climber() {
    motor = new TalonFX(CLIMBER_MOTOR_ID);
    motor.getConfigurator().apply(CLIMBER_CONFIG);
    inputs = new ClimberInputsAutoLogged();
    motor.setPosition(0);
  }

  /**
   * Sets motor speed using duty cycle out
   * @param speed Motor power from -1 to 1
   */
  private void set(double speed){
    motor.set(speed);
  }

  /**
   * runs the climber motor upwards unless the climber has reached the soft limit, which is relative to the climbers start position when code was deployed
   */
  public void climberUp() {
    desiredSpeed = -0.5;
  }
  /**
   * runs the climber motor upwards unless the climber has reached the soft limit, which is relative to the climbers start position when code was deployed
   */
  public void climberDown() {
    desiredSpeed = 0.5;
  }

  /**
   * Sets motor power to zero
   */
  public void stop(){
    desiredSpeed = 0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    inputs.motor.log(motor);
    Logger.processInputs("Climber", inputs);

    if(desiredSpeed > 0 && motor.getPosition().getValueAsDouble() > CLIMBER_MAX_POSITION) {
      motor.stopMotor();
    } else {
      motor.set(desiredSpeed);
    }
  }
}
