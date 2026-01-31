// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.settings.ClimberState;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.RobotState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutomaticClimb extends SequentialCommandGroup {
  /** moves to the nearest climbing position and completes a climb sequence */
  public AutomaticClimb(DrivetrainSubsystem drivetrain, Climber climber) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(()->climber.climberUp(), climber),
      new MoveToClimbingPose(drivetrain),
      new WaitUntilCommand(()->RobotState.getInstance().climberState == ClimberState.Up),
      new MoveMeters(drivetrain, 0.05, 0.5, 0, 0),
      new ClimberArmDown(climber)
    );
  }
}
