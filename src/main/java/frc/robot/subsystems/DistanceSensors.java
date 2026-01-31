// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import static frc.robot.settings.Constants.HopperConstants.DISTANCE_SENSOR_RANGE;

import org.littletonrobotics.junction.Logger;

import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LogInputs.DistanceSensorsInputs;
import frc.robot.LogInputs.DistanceSensorsInputsAutoLogged;
import frc.robot.settings.HopperState;
import frc.robot.settings.Constants.HopperConstants;

public class DistanceSensors extends SubsystemBase {
  /** Creates a new DistanceSensors. */
  TimeOfFlight fuelStock;
  DistanceSensorsInputsAutoLogged inputs;
    
  public DistanceSensors() {
   fuelStock = new TimeOfFlight(HopperConstants.TOP_LEFT_SENSOR_ID);
   inputs = new DistanceSensorsInputsAutoLogged();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    inputs.fuelStockRange = fuelStock.getRange();

    Logger.processInputs("DistanceSensor", inputs);
    if(inputs.fuelStockRange>DISTANCE_SENSOR_RANGE){
      RobotState.getInstance().hopperState = HopperState.Empty;
    }else if(RobotState.getInstance().fullSwitchTriggered){
      RobotState.getInstance().hopperState = HopperState.Full;
    }else{
      RobotState.getInstance().hopperState = HopperState.HalfFull;
    }
      
    
  }

}
