// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.RobotState;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LightsCommand extends Command {
  Lights lights;
  int loopsRan;

  /** Creates a new LightsCommand. */
  public LightsCommand(Lights lights) {
    this.lights = lights;
    addRequirements(lights);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    loopsRan = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Boolean hubActive = RobotState.getInstance().hubActive();

    if (DriverStation.isAutonomous()) {
      return;
    }
    loopsRan++;
    if (loopsRan < 10) {
      return;
    } else {
      loopsRan = 0;
    }

    if (hubActive) {
      lights.setLights(0, 60, 0, 0, 255);
    } else if (RobotState.getInstance().LimelightsUpdated) {
      lights.setLights(0, 60, 0, 255, 0); 
    } else if (RobotState.getInstance().LimelightsUpdated && hubActive) {
      lights.setLights(0, 60, 255, 0, 255); 
    } else {
        lights.setLights(0, 60, 255, 255, 255);
      }
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    lights.lightsOut();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
