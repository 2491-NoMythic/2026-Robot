// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import static frc.robot.settings.Constants.DriveConstants.AUTO_AIM_ROBOT_kD;
import static frc.robot.settings.Constants.DriveConstants.MAX_VELOCITY_METERS_PER_SECOND;
import static frc.robot.settings.Constants.DriveConstants.ROBOT_ANGLE_TOLERANCE;
import static frc.robot.settings.Constants.Field.BLUE_HUB_COORDINATE;
import static frc.robot.settings.Constants.Field.RED_HUB_COORDINATE;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RobotState;
import frc.robot.subsystems.DrivetrainSubsystem;
import static frc.robot.settings.Constants.ShooterConstants.*;
import static frc.robot.settings.Constants.Vision.MAX_TAG_DISTANCE;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AimRobot extends Command {
  /** Creates a new AimRobotMoving. */
  PIDController rotationController = new PIDController(AUTO_AIM_ROBOT_kP, AUTO_AIM_ROBOT_kI, AUTO_AIM_ROBOT_kD);
  DrivetrainSubsystem drivetrain;

  DoubleSupplier joystickXSupplier;
  DoubleSupplier joystickYSupplier;
  DoubleSupplier targetSupplier;

  /**
   * Aims the robot using an angle supplier
   * @param drivetrain
   * @param joystickXSupplier
   * @param joystickYSupplier
   * @param targetSupplier Supplier for angle, in degrees
   */
  public AimRobot(DrivetrainSubsystem drivetrain, DoubleSupplier joystickXSupplier, DoubleSupplier joystickYSupplier, DoubleSupplier targetSupplier) {
    this.drivetrain = drivetrain;
    this.joystickXSupplier = joystickXSupplier;
    this.joystickYSupplier = joystickYSupplier;
    this.targetSupplier = targetSupplier;
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
    double target = targetSupplier.getAsDouble();
    rotationController.setSetpoint(target); //this is in degrees
    SmartDashboard.putNumber("Desired robot angle", target);
    Logger.recordOutput("Blue hub", BLUE_HUB_COORDINATE);
    Logger.recordOutput("Red hub", RED_HUB_COORDINATE);
    double rotationSpeed = rotationController.calculate(drivetrain.getOdometryRotation().getDegrees());

    int invert;
    if (DriverStation.getAlliance().get() == Alliance.Red) {
      invert = -1;
    } else {
      invert = 1;
    }
    drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
      joystickYSupplier.getAsDouble() * invert * MAX_VELOCITY_METERS_PER_SECOND,
      joystickXSupplier.getAsDouble() * invert * MAX_VELOCITY_METERS_PER_SECOND,
      rotationSpeed,
      drivetrain.getOdometryRotation()));
    SmartDashboard.putNumber("angle targeting error", rotationController.getError());
    RobotState.getInstance().Aimed = rotationController.getError() < 2;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotState.getInstance().Aimed = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
