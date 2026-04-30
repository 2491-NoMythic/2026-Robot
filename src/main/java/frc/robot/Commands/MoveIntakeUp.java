// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import static frc.robot.settings.Constants.IntakeConstants.INTAKE_SPEED_RPS;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.RobotState;
import frc.robot.settings.Constants.IntakeConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveIntakeUp extends Command {
  Intake intake;
  Timer timer;
  double timeAfterFirstRaise;
 
  public MoveIntakeUp(Intake intake, double timeAfterFirstRaise) {
    this.intake = intake;
    this.timeAfterFirstRaise = timeAfterFirstRaise;
    timer = new Timer();
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
    System.out.println("MoveIntakeUpRunning");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(timer.get() < timeAfterFirstRaise) {
      intake.feedHopper();
      intake.setIntakeAngle(-0.17);
    } else {
      intake.setIntakeAngle(-0.3);
    }
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotState.getInstance().feedingShooter = true;
    intake.deployIntake();
    intake.stopWheels();
    timer.stop();
    timer.reset();
    System.out.println("MoveIntakeUpStopped");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
