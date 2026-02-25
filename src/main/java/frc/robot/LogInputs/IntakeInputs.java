// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.LogInputs;

import org.littletonrobotics.junction.AutoLog;

@AutoLog
/** Add your docs here. */
public class IntakeInputs {
    public static final boolean IntakeInputs = false;
    public MotorLoggerInputsAutoLogged wheelsMotor = new MotorLoggerInputsAutoLogged();
    public MotorLoggerInputsAutoLogged deployerMotor = new MotorLoggerInputsAutoLogged();
}
