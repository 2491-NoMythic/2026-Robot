// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import static frc.robot.settings.Constants.ShooterConstants.SHOOTING_SPEED_RPS;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.settings.Constants.AimAtLocationConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Shooter; 

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AimAtLocation extends ParallelCommandGroup {
  public enum Location{
    Hub,
    LeftTrench,
    RightTrench,
    Tower,
    LeftCorner,
    RightCorner,
  }

  static double getTargetHoodAngle(Location location){
    switch (location) {
      case Hub: return AimAtLocationConstants.HUB_HOOD_ANGLE;
      case Tower: return AimAtLocationConstants.TOWER_HOOD_ANGLE;
      case LeftTrench: 
      case RightTrench:
        return AimAtLocationConstants.TRENCH_HOOD_ANGLE;
      case LeftCorner:
      case RightCorner:
        return AimAtLocationConstants.CORNER_HOOD_ANGLE;
      default: return 0;
    }
  }

  static double getTargetRobotAngle(Location location){
    if (DriverStation.getAlliance().get() == Alliance.Red){
      switch (location) {
        case Hub:         return AimAtLocationConstants.HUB_ROBOT_ANGLE + 180;
        case LeftTrench:  return AimAtLocationConstants.L_TRENCH_ROBOT_ANGLE + 180;
        case RightTrench: return AimAtLocationConstants.R_TRENCH_ROBOT_ANGLE + 180;
        case Tower:       return AimAtLocationConstants.TOWER_ROBOT_ANGLE + 180;
        case LeftCorner:  return AimAtLocationConstants.L_CORNER_ROBOT_ANGLE + 180;
        case RightCorner: return AimAtLocationConstants.R_CORNER_ROBOT_ANGLE + 180;
        default: return 0;
      }
    }

    switch (location) {
      case Hub:         return AimAtLocationConstants.HUB_ROBOT_ANGLE;
      case LeftTrench:  return AimAtLocationConstants.L_TRENCH_ROBOT_ANGLE;
      case RightTrench: return AimAtLocationConstants.R_TRENCH_ROBOT_ANGLE;
      case Tower:       return AimAtLocationConstants.TOWER_ROBOT_ANGLE;
      case LeftCorner:  return AimAtLocationConstants.L_CORNER_ROBOT_ANGLE;
      case RightCorner: return AimAtLocationConstants.R_CORNER_ROBOT_ANGLE;
      default: return 0;
    }
  }

  /** Creates a new AimAtLocation. */
  public AimAtLocation(DrivetrainSubsystem drivetrain, Shooter shooter, DoubleSupplier joystickXSupplier, DoubleSupplier joystickYSupplier, Location location) {
    double hoodAngle = getTargetHoodAngle(location);
    double shootSpeed = SHOOTING_SPEED_RPS;
    if(location == Location.LeftCorner || location == Location.RightCorner) {
      shootSpeed = AimAtLocationConstants.CORNER_SHOOTING_SPEED;
    }
    if(location == Location.Hub) {
      shootSpeed = AimAtLocationConstants.HUB_SHOOTING_SPEED;
    }
    addCommands(
      new AimHoodFixed(shooter, hoodAngle, false, shootSpeed),
      new AimRobot(drivetrain, joystickXSupplier, joystickYSupplier, ()-> getTargetRobotAngle(location)));
  }
}
