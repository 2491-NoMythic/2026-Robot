// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveToClimbingPose extends Command {
  DrivetrainSubsystem drivetrain;
  Pose2d targetPose;
  int cyclesGood;
  /** Creates a new MoveToClimbingPose. */
  public MoveToClimbingPose(DrivetrainSubsystem drivetrain) {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    cyclesGood = 0;

    if (DriverStation.getAlliance().get() == Alliance.Red){
      if (drivetrain.getPose().getY() > 4.3){
        targetPose = new Pose2d(15.5, 5.2, new Rotation2d(Math.PI));
      } else {
        targetPose = new Pose2d(15.5, 3.4, new Rotation2d(0));
      }
    } else {
      if (drivetrain.getPose().getY() > 3.73){
        targetPose = new Pose2d(1.1, 4.6, new Rotation2d(Math.PI));
      } else {
        targetPose = new Pose2d(1.1, 2.9, new Rotation2d(0));
      }
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.moveTowardsPose(targetPose);
    
    if(drivetrain.getPositionTargetingError() < 0.015) {
      cyclesGood++;
    } else {
      cyclesGood = 0;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    cyclesGood = 0;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return cyclesGood > 3 && drivetrain.isAtRotationTarget();
  }
}
