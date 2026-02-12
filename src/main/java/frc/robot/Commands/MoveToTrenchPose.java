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
public class MoveToTrenchPose extends Command {
  DrivetrainSubsystem drivetrain;
  Pose2d targetPose;
  int cyclesGood;
  /** Creates a new MoveToTrenchPose. */
  public MoveToTrenchPose(DrivetrainSubsystem drivetrain) {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    cyclesGood = 0; //up is 7.425 between 3 nd 5 down is 0.650

    double poseX = drivetrain.getPose().getX();
    double poseY = drivetrain.getPose().getY();
    double halfMargin = 1;
    double leftTrenchY = 7.425;
    double rightTrenchY = 0.650;
    if (poseX > 3 && poseX < 5){
      if (poseY > leftTrenchY - halfMargin && poseY < leftTrenchY + halfMargin){
        targetPose = new Pose2d(poseX, leftTrenchY, drivetrain.getPose().getRotation());
      } else if (poseY > rightTrenchY - halfMargin && poseY < rightTrenchY + halfMargin ) {
        targetPose = new Pose2d(poseX, rightTrenchY, new Rotation2d(0));
      }
    } else if (poseX > 10.5 && poseX < 13.5) {
      if (poseY > leftTrenchY - halfMargin && poseY < leftTrenchY + halfMargin){
        targetPose = new Pose2d(poseX, leftTrenchY, drivetrain.getPose().getRotation());
      } else if (poseY > rightTrenchY - halfMargin && poseY < rightTrenchY + halfMargin ) {
        targetPose = new Pose2d(poseX, rightTrenchY, new Rotation2d(0));
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
