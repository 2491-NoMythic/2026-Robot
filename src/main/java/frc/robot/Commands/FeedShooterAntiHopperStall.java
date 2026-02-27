// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import static frc.robot.settings.Constants.HopperConstants.HOPPER_ROLLER_SPEED;

import java.util.Map;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Indexer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FeedShooterAntiHopperStall extends ParallelCommandGroup {
  /** 
   * This command will do two things at once:
   * 1) run the indexer at the shooter feeding speed
   * 2) Repeatedly run a SelectCommand that checks if the hopper is stalling, then either runs the hopper at the
   * indexer feeding speed, or runs the hopper backwards for a moment to try and unstick whatever is stalling the motor, then
   * runs the hopper at indexer feeding speed
  */
  public FeedShooterAntiHopperStall(Hopper hopper, Indexer indexer) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(()->indexer.feedShooter(), indexer),
      new RepeatCommand(new SelectCommand<>(
        Map.ofEntries(
          Map.entry(true, new SequentialCommandGroup(
            new InstantCommand(()->hopper.setHopperRoller(-0.3)),
            new WaitCommand(1),
            new InstantCommand(()->hopper.feedIndexer(), hopper)
          )),
          Map.entry(false, new InstantCommand(()->hopper.feedIndexer(), hopper))
        ),
        ()->hopper.isStalled()))
    );
  }
}
