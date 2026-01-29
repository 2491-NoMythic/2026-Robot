// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.LogInputs;

import org.littletonrobotics.junction.AutoLog;

@AutoLog
/** Add your docs here. */
public class HopperInputs {
    public MotorLoggerInputsAutoLogged wideMotorInput = new MotorLoggerInputsAutoLogged();
    public MotorLoggerInputsAutoLogged tallMotorInput = new MotorLoggerInputsAutoLogged();
    // public boolean hopperWideInput;
    // public boolean wideDesiredInput;
    // public boolean hopperTallInput;
    // public boolean tallDesiredInput;
    public boolean hopperExpandedInput;
}
