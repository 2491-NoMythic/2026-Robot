// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.LogInputs;

import org.littletonrobotics.junction.AutoLog;

@AutoLog
/** Add your docs here. */
public class IntakeInputs {
    public MotorLoggerInputsAutoLogged wheelsMotorLead = new MotorLoggerInputsAutoLogged();
    public MotorLoggerInputsAutoLogged wheelsMotorFollow = new MotorLoggerInputsAutoLogged();
    public MotorLoggerInputsAutoLogged deployerMotor = new MotorLoggerInputsAutoLogged();
    public boolean forwardLimitSwitch;
    public boolean reverseLimitSwitch;
    public double encoderPosition;
}
