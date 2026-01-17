// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import static frc.robot.settings.Constants.DriveConstants.AUTO_AIM_ROBOT_kD;
import static frc.robot.settings.Constants.DriveConstants.ROBOT_ANGLE_TOLERANCE;
import static frc.robot.settings.Constants.Field.BLUE_HUB_COORDINATE;
import static frc.robot.settings.Constants.Field.RED_HUB_COORDINATE;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;
import static frc.robot.settings.Constants.ShooterConstants.*;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AimRobotMoving extends Command {
  /** Creates a new AimRobotMoving. */
  PIDController rotationController = new PIDController(AUTO_AIM_ROBOT_kP, AUTO_AIM_ROBOT_kI, AUTO_AIM_ROBOT_kD);
  DrivetrainSubsystem drivetrain;

  DoubleSupplier joystickXSupplier;
  DoubleSupplier joystickYSupplier;

  public AimRobotMoving(DrivetrainSubsystem drivetrain, DoubleSupplier joystickXSupplier, DoubleSupplier joystickYSupplier) {
    this.drivetrain = drivetrain;
    this.joystickXSupplier = joystickXSupplier;
    this.joystickYSupplier = joystickYSupplier;
    addRequirements(drivetrain);
    rotationController.setTolerance(ROBOT_ANGLE_TOLERANCE);
    rotationController.enableContinuousInput(-180, 180);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    rotationController.setSetpoint(drivetrain.getDesiredRobotAngle());
    SmartDashboard.putNumber("Desired robot angle", drivetrain.getDesiredRobotAngle());
    Logger.recordOutput("Blue hub", BLUE_HUB_COORDINATE);
    Logger.recordOutput("Red hub", RED_HUB_COORDINATE);
    double rotationSpeed = rotationController.calculate(drivetrain.getOdometryRotation().getDegrees());
    double allianceOffset;
    if(DriverStation.getAlliance().get() == Alliance.Red) {
      allianceOffset = Math.PI;
    } else {
      allianceOffset = 0;
    }
    drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
      joystickXSupplier.getAsDouble(),
      joystickYSupplier.getAsDouble(),
      rotationSpeed,
      new Rotation2d(drivetrain.getOdometryRotation().getRadians()+allianceOffset)));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
