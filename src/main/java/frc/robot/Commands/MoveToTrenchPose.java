// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import static edu.wpi.first.units.Units.Rotation;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveToTrenchPose extends Command {
  DrivetrainSubsystem drivetrain;
  DoubleSupplier xMovementSupplier;
  Pose2d targetPose;
  int cyclesGood;
  /** Creates a new MoveToTrenchPose. */
  public MoveToTrenchPose(DrivetrainSubsystem drivetrain, DoubleSupplier xMovementSupplier) {
    this.drivetrain = drivetrain;
    this.xMovementSupplier = xMovementSupplier;
    addRequirements(drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    cyclesGood = 0; //up is 7.425 between 3 nd 5 down is 0.650

    double poseX = drivetrain.getPose().getX();
    double poseY = drivetrain.getPose().getY();

    double leftTrenchY = 7.425;
    double rightTrenchY = 0.650;
    double yFieldCenter = 4.000;

    Rotation2d newRotation = Rotation2d.fromDegrees(Math.round( drivetrain.getPose().getRotation().getDegrees()/180 ) * 180);

    if (poseY > yFieldCenter){
      targetPose = new Pose2d(poseX, leftTrenchY, newRotation);
    } else if (poseY <= yFieldCenter) {
      targetPose = new Pose2d(poseX, rightTrenchY, newRotation);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.moveTowardsTrenchPose(xMovementSupplier, targetPose);
    
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
