// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.settings.Constants.LightConstants;
import frc.robot.settings.EffectEnums;
import frc.robot.settings.LightsEnums;

import com.ctre.phoenix6.signals.RGBWColor;

public class Lights extends SubsystemBase {
  private AddressableLED lights;
  private AddressableLEDBuffer LEDBuffer;
  LightsEnums lightsToBlink;

  int blinkedRed;
  int blinkedGreen;
  int blinkedBlue;
  boolean blinkLights;

  public RGBWColor[] lightColors = new RGBWColor[LightConstants.ALL_LIGHT_END];

  Timer timer;
  public double lastTime;
  public double deltaTime;
  public double lastSparkle;

  public EffectEnums currentEffect;
  SendableChooser<EffectEnums> effectChooser;

  /** Creates a new Lights. */
  public Lights() {
    lights = new AddressableLED(9);
    lights.setLength(82);
    LEDBuffer = new AddressableLEDBuffer(82);

    lights.start();
    timer = new Timer();
    timer.start();

    for (int i = 0; i < LightConstants.ALL_LIGHT_END; i++) {
        lightColors[i] = new RGBWColor(255, 255, 255);
    }

    SmartDashboard.putNumber("shooterMult", 0.1);
    SmartDashboard.putNumber("Lights/Brightness", 1);
    SmartDashboard.putBoolean("Lights/Show Mode", false);
    effectChooser = new SendableChooser<EffectEnums>();
    effectChooser.addOption("RGB", EffectEnums.RGBPride);
    effectChooser.addOption("Flow", EffectEnums.IndexingFlow);
    effectChooser.addOption("Green", EffectEnums.ShutdownGreen);
    effectChooser.addOption("Breathe", EffectEnums.AllianceBreathe);
    effectChooser.addOption("Flash", EffectEnums.RangeFlash);
    effectChooser.addOption("None", null);
    effectChooser.setDefaultOption("None", null);
    SmartDashboard.putData("Lights/Effect Chooser", effectChooser);
}

  public void setOneLightRGB(int index, int R, int G, int B) {
    double brightness = SmartDashboard.getNumber("Lights/Brightness", 1);
    LEDBuffer.setRGB(index, (int)(R * brightness), (int)(G * brightness), (int)(B * brightness));
    //} catch(Exception e) {}
  }

  public void setLights(int start, int end, int R, int G, int B) {
    for (int i = start; i < end; i++) {
      setOneLightRGB(i, R, G, B);
    }

    lights.setData(LEDBuffer);
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

  @SuppressWarnings("unused")
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

  public void allLights(LightsEnums lightsEnums) {
    for (int index = 0; index < LightConstants.ALL_LIGHT_END; index++) {
      lightColors[index] = new RGBWColor(255, 0, 255);
    }

  }

  @Override
  public void periodic() {
    //updateBlinkedLights();
    // This method will be called once per scheduler run
    double velocity = 0;

    if(RobotState.getInstance().lightsRobotDisabled) {
      
      if(SmartDashboard.getBoolean("Lights/Show Mode", false)) { //show mode
        currentEffect = EffectEnums.RGBPride;
      } else {
        currentEffect = EffectEnums.AllianceBreathe;
      }
    } else if(DriverStation.isAutonomous()) { //in autonomous
      currentEffect = EffectEnums.RGBPride;
    } else if(RobotState.getInstance().lightsShooterOutOfRange) {
      currentEffect = EffectEnums.RangeFlash;
    } else if(RobotState.getInstance().lightsIndexing) {
      currentEffect = EffectEnums.IndexingFlow;
    } else {
      velocity = RobotState.getInstance().lightsRobotSpeed;
      currentEffect = EffectEnums.SpeedGlow;
    }

    if(effectChooser.getSelected() != null){
      currentEffect = effectChooser.getSelected();
    }
    

    double time = timer.get();
    deltaTime = time - lastTime;
    lastTime = time;

    int brightness = 0;

    switch (currentEffect) {
      case AllianceBreathe:
        brightness = (int) ((Math.sin(time * 2) + 1) / 2 * 255);
        if(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red){
          setSystemLights(LightsEnums.All, 255, 0, brightness);
        } else if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue){
          setSystemLights(LightsEnums.All, brightness, 0, 255);
        } else {
          setSystemLights(LightsEnums.All, 255, brightness, 255);
        }

        break;

      case IndexingFlow:
        for(int i = 0; i < LightConstants.ALL_LIGHT_END; i++){
          brightness = (int) ((Math.sin(time * 32 + (i)) + 1) / 2 * 255);
          brightness = Math.max(brightness, 0);
          LEDBuffer.setRGB(i, brightness, 0, brightness);
        }
        lights.setData(LEDBuffer);

        break;

      case RangeFlash:
        brightness = (int) (Math.round((Math.sin(time * 6) + 1)/2)) * 255;
        setSystemLights(LightsEnums.All, brightness, 0, 0);

        break;

      case ShootingSparkle:
        if(time - lastSparkle > SmartDashboard.getNumber("shooterMult", 0.1)) {
          int index = (int)(Math.random() * (LightConstants.ALL_LIGHT_END - 2) + 1);
          LEDBuffer.setRGB(index, 255, 255, 255);
          LEDBuffer.setRGB(index - 1, 255, 170, 255);
          LEDBuffer.setRGB(index + 1, 255, 170, 255);
          lastSparkle = time;
        }

        for(int i = 0; i < LightConstants.ALL_LIGHT_END; i++){
          brightness = (int) ((Math.sin(time * 8 + (i * i * i/2)) + 1) / 2 * 255);
          brightness = (int)(brightness/1.33) + (int)(255 * 0.25);
          //brightness = 150;
          double averageGreen = ( (LEDBuffer.getGreen(Math.max(i - 1, LightConstants.ALL_LIGHT_START)) + LEDBuffer.getGreen(i) + LEDBuffer.getGreen(Math.min(i + 1, LightConstants.ALL_LIGHT_END - 1)))/3 );
          //double averageGreen = Math.max(Math.max(LEDBuffer.getGreen(Math.max(i, LightConstants.ALL_LIGHT_START)), LEDBuffer.getGreen(i)),  LEDBuffer.getGreen(Math.min(i, LightConstants.ALL_LIGHT_END)));
          double modifiedGreen = (LEDBuffer.getGreen(i) + (averageGreen - LEDBuffer.getGreen(i)) * deltaTime * 5);
          LEDBuffer.setRGB(i, brightness, (int)(modifiedGreen * Math.pow(0.25, deltaTime)), brightness);
        }
        lights.setData(LEDBuffer);

        break;

        case ShutdownGreen:
        brightness = (int) (Math.round((Math.sin(time * 4) + 1)/2)) * 120;
        setSystemLights(LightsEnums.All, 0, 135 + brightness, 0);

        break;

        case SpeedGlow:
        brightness = (int)(((velocity / 4) * 255 * 2));
        brightness = (int)(brightness * brightness / (255 * 2));
        if(brightness > 255) { //when going extra fast the lights "overclock" into turning white
          setSystemLights(LightsEnums.All, brightness, brightness - 255, brightness);
        } else {
          setSystemLights(LightsEnums.All, brightness, 0, brightness);
        }

        break;

        case RGBPride:
          for (int i = 0; i < LightConstants.ALL_LIGHT_END; i++) {
            float rgbHue = (float)(time + i / 80);
            java.awt.Color rgbValues = new java.awt.Color(java.awt.Color.HSBtoRGB(rgbHue, 1, 1));
            LEDBuffer.setRGB(i, rgbValues.getRed(), rgbValues.getGreen(), rgbValues.getBlue());
          }
          lights.setData(LEDBuffer);

          break;
    }
    
  }
}
