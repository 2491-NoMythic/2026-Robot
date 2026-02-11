// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Shooter;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AimAtLocation extends ParallelCommandGroup {
  public enum Location{
    Hub,
    Trench,
    Tower
  }
  static double getHoodAngle(Location location){
    switch (location) {
      case Hub: return 2491;
      case Trench: return 2491;
      case Tower: return 2491;
    }
    return 0;
  }
  static double getRobotAngle(Location location){
    switch (location) {
      case Hub: return 2491;
      case Trench: return 2491;
      case Tower: return 2491;
    }
    return 0;
  }
  /** Creates a new AimAtLocation. */
  public AimAtLocation(DrivetrainSubsystem drivetrain, Shooter shooter, DoubleSupplier joystickXSupplier, DoubleSupplier joystickYSupplier, Location location) {
    double hoodAngle = getHoodAngle(location);
    double robotAngle = getRobotAngle(location);
    addCommands(new AimHoodFixed(shooter, hoodAngle, false),
    new AimRobot(drivetrain, joystickXSupplier, joystickYSupplier, ()->robotAngle));

  }
}
