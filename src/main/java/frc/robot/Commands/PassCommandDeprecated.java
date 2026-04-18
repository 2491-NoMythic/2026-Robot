// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import static frc.robot.settings.Constants.ShooterConstants.SHOOTING_SPEED_RPS;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RobotState;
import frc.robot.subsystems.Shooter;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PassCommandDeprecated extends Command {
  Shooter shooter;
  /** Creates a new PassCommand. */
  public PassCommandDeprecated(Shooter shooter) {
    this.shooter = shooter;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double x = RobotState.getInstance().robotPosition.getX();
    boolean isInNeutralZone = x > 5 && x < 11;
    if (isInNeutralZone) {
      shooter.setVelocity(51);
      shooter.setDesiredHoodAngle(35);
    } else {
      shooter.setVelocity(65);
      shooter.setDesiredHoodAngle(40);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setVelocity(SHOOTING_SPEED_RPS);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
