// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.configs.CANdleFeaturesConfigs;
import com.ctre.phoenix6.configs.CustomParamsConfigs;
import com.ctre.phoenix6.configs.LEDConfigs;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.BridgeOutputValue;
import com.ctre.phoenix6.signals.StripTypeValue;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.settings.Constants.DriveConstants;
import frc.robot.settings.Constants.LightConstants;

public class Lights extends SubsystemBase {
  CANdle candle;
  CANdleConfiguration candleConfig;
  private AddressableLED lights;
  private AddressableLEDBuffer LEDBuffer;

  /** Creates a new Lights. */
  public Lights() {
  lights = new AddressableLED(6);
  lights.setLength(60);
  LEDBuffer = new AddressableLEDBuffer(60);
  candle = new CANdle(LightConstants.CANDLE_ID);
    candleConfig = new CANdleConfiguration();
    //CANdleConfig = CANdleConfig.withLED(LEDConfigs.withBrightnessScalar(1.0));
  LEDConfigs ledConfigs = new LEDConfigs();
  ledConfigs = ledConfigs.withBrightnessScalar(1).withStripType(StripTypeValue.GRB);
  candleConfig.withLED(ledConfigs);
  candle.getConfigurator().apply(candleConfig);
  }

  public void setOneLightRGB(int index, int R, int G, int B) {
    LEDBuffer.setRGB(index, R, G, B);
  }

  public void setLights(int start, int end, int R, int G, int B) {
    for (int i = start; i < end; i++) {
      setOneLightRGB(i, R, G, B);
    }
  }

  public void lightsOut() {
    setLights(0, LEDBuffer.getLength(), 0, 0, 0);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
