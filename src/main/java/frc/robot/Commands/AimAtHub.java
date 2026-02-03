// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AimAtHub extends ParallelCommandGroup {
  /** Creates a new AimAtHub. */
  AimHood aimHood;
  AimRobotMoving aimRobotMoving;
  DrivetrainSubsystem drivetrain;
  Shooter shooter;

  DoubleSupplier joystickXSupplier;
  DoubleSupplier joystickYSupplier;
  public AimAtHub(AimAtHub aimAtHub, AimHood aimHood, DrivetrainSubsystem drivetrain, Shooter shooter, DoubleSupplier joystickXSupplier, DoubleSupplier joystickYSupplier) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new InstantCommand(() -> drivetrain.updateDesiredRobotAngle(), drivetrain),
    new AimRobotMoving(drivetrain, joystickXSupplier, joystickYSupplier),
    new AimHood(shooter));
  }
}
