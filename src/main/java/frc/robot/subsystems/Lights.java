// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.settings.Constants.LightConstants;
import frc.robot.settings.LightsEnums;

public class Lights extends SubsystemBase {
  private AddressableLED lights;
  private AddressableLEDBuffer LEDBuffer;
  Timer timer;
  LightsEnums lightsToBlink;

  int blinkedRed;
  int blinkedGreen;
  int blinkedBlue;
  boolean blinkLights;

  /** Creates a new Lights. */
  public Lights() {
    lights = new AddressableLED(6);
    lights.setLength(30);
    LEDBuffer = new AddressableLEDBuffer(30);
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

  public void setSystemLights(LightsEnums lightEnums, int R, int G, int B) {
    switch (lightEnums) {
      case All:
        setLights(LightConstants.ALL_LIGHT_START, LightConstants.ALL_LIGHT_END, R, G, B);
        break;
    }
  }

  public void activeEnding() {
    String phase = RobotState.getPhase();
    boolean hubActive = RobotState.hubActive();
    int phaseTime = RobotState.getPhaseTimeLeft();
    if (phaseTime < 5) {
      if (hubActive) {
        setLights(0, LEDBuffer.getLength(), 100, 50, 0);
      } else if (!hubActive && phase != "SHIFT 4") {
        setLights(0, LEDBuffer.getLength(), 200, 50, 0);
      }
    }
  }

  public void blinkLights(LightsEnums lightsEnums, int R, int G, int B) {
    timer.start();
    lightsToBlink = lightsEnums;
    blinkedBlue = B;
    blinkedGreen = G;
    blinkedRed = R;
    blinkLights = true;
  }

  public void stopBlinkingLights() {
    blinkLights = false;
    timer.stop();
    timer.reset();
  }

  private void updateBlinkedLights() {
    if (blinkLights) {
      if (timer.get() < 0.1) {
        setSystemLights(lightsToBlink, blinkedRed, blinkedGreen, blinkedBlue);
      } else if (timer.get() < 1) {
        setSystemLights(lightsToBlink, 0, 0, 0);
      } else {
        timer.reset();
      }
    }
  }

  public void breathingLights(LightsEnums lightsEnums, int R, int G, int B) {
    double time = timer.get();
    int brightness = (int) ((Math.sin(time * 2) + 1) / 2 * 255);
    setSystemLights(lightsEnums, (int) (R * brightness / 255.0), (int) (G * brightness / 255.0),
        (int) (B * brightness / 255.0));
  }

  @Override
  public void periodic() {
    updateBlinkedLights();
    // This method will be called once per scheduler run
  }
}