// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Climber;
import frc.robot.Commands.ClimberArmUp;
import frc.robot.Commands.ClimberArmDown;
import frc.robot.subsystems.DrivetrainSubsystem;
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Climb extends SequentialCommandGroup {
  Climber climber;
  DrivetrainSubsystem drivetrain;
  /** Creates a new Climb. */
  public Climb(Climber climber, DrivetrainSubsystem drivetrain) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new ClimberArmUp(climber),
     new InstantCommand(()->drivetrain.drive(new ChassisSpeeds(0.5,0,0))),
     new WaitCommand(0.5), new InstantCommand(()->drivetrain.drive(new ChassisSpeeds(0,0,0))),
     new ClimberArmDown(climber),
     new InstantCommand(()->drivetrain.drive(new ChassisSpeeds(-0.5,0,0))),
     new WaitCommand(0.5),
     new InstantCommand(()->drivetrain.drive(new ChassisSpeeds(0,0,0))));
  }
}
