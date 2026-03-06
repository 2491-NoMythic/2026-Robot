// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.RobotState;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
      new InstantCommand(()-> System.out.println("start")),
      new RunCommand(()-> drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(speedOverBump, 0, 0), drivetrain.getPose().getRotation())), drivetrain).withDeadline(new WaitCommand(0.5)),
      new WaitCommand(1),
      new InstantCommand(()-> System.out.println("rotating")),
      new InstantCommand(()->drivetrain.turnToSeeAprilTag(), drivetrain),
      new InstantCommand(()-> System.out.println("wait for limelight update")),
      new WaitUntilCommand(()->RobotState.getInstance().LimelightsUpdated),
      new WaitCommand(0.4),
      new InstantCommand(()-> System.out.println("end"))
    );
  }
}
