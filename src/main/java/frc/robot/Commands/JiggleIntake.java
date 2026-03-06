// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class JiggleIntake extends Command {
  Intake intake;
  Timer timer;
  /** Creates a new JiggleIntake. */
  public JiggleIntake(Intake intake) {
    this.intake = intake;
    timer = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    intake.feedHopper();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double intakeSpeed = -(3 * Math.sin(Math.toDegrees(timer.get())) + 0.7);
    intake.setDeployer(intakeSpeed);
    SmartDashboard.putNumber("sinewave intake speed", intakeSpeed);
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopDeployer();
    intake.stopWheels();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
