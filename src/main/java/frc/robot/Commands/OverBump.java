// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.RobotState;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OverBump extends SequentialCommandGroup {
  /** Creates a new OverBump. */
  public OverBump(DrivetrainSubsystem drivetrain, double speedOverBump) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(()-> drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(0, speedOverBump, 0), drivetrain.getGyroscopeRotation())), drivetrain),
      new InstantCommand(()->System.out.println("OVERBUMP:started driving")),
      new WaitUntilCommand(()->Math.abs(drivetrain.getPigeonPitch()) > 5 || Math.abs(drivetrain.getPigeonRoll()) > 5),
      new InstantCommand(()->System.out.println("OVERBUMP:going over bump")),
      new WaitUntilCommand(1),
      new InstantCommand(()->drivetrain.drive(new ChassisSpeeds(0, 0, 1)), drivetrain),
      new InstantCommand(()->System.out.println("OVERBUMP:spinning")),
      new WaitUntilCommand(()->RobotState.getInstance().LimelightsUpdated)
    );
  }
}
