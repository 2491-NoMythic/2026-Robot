// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import static frc.robot.settings.Constants.ShooterConstants.SHOOTING_SPEED_RPS;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.settings.Constants.ShooterConstants;
import frc.robot.subsystems.Shooter;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AimHoodFixed extends Command {
  Shooter shooter;
  double angle;
  double shooterSpeed;
  /** Creates a new AimHoodFixed. */
  public AimHoodFixed(Shooter shooter, double angle) {
    this(shooter, angle, SHOOTING_SPEED_RPS);
  }

  public AimHoodFixed(Shooter shooter, double angle, double speed) {
    this.shooter = shooter;
    this.angle = angle;
    this.shooterSpeed = speed;
    addRequirements(shooter);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setDesiredHoodAngle(angle);
    shooter.setVelocity(shooterSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setDesiredHoodAngle(ShooterConstants.HOOD_DOWN_POSITION);
    shooter.setVelocity(SHOOTING_SPEED_RPS);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
