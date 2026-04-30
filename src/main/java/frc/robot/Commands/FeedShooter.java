// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import static frc.robot.settings.Constants.IntakeConstants.INTAKE_SPEED_RPS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.RobotState;
import frc.robot.settings.Constants.IntakeConstants;
import frc.robot.subsystems.Hopper;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class FeedShooter extends Command {
  Indexer indexer;
  Hopper hopper;
  Intake intake;
  Timer timer;
  Command moveIntakeUp;
  
  /** Creates a new RunIndexer. */
  public FeedShooter(Indexer indexer, Hopper hopper, Intake intake) {
    this.indexer = indexer;
    this.hopper = hopper;
    this.intake = intake;
    timer = new Timer();
    addRequirements(hopper, indexer); //addRequirements(hopper, indexer, intake);
    SmartDashboard.putBoolean("shooterOverride", false);
    moveIntakeUp = new MoveIntakeUp(intake, 1);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotState.getInstance().feedingShooter = true;
    timer.start();
    System.out.println("FeedShooterRunning");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(timer.get() < 0.1) {
      indexer.set(-0.5);
      hopper.setHopperRoller(-0.4);
    } else if(timer.get() < 2.2){
      indexer.feedShooter();
      hopper.feedIndexer();
    } else if(timer.get() < 3.2) {
      //intake.feedHopper();
      //intake.setIntakeAngle(-0.13);
    } else {
      //intake.setIntakeAngle(-0.3);
    }
    if(timer.get() > 2.2 && intake.getCurrentCommand() == null){
      CommandScheduler.getInstance().schedule(moveIntakeUp);
    }
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotState.getInstance().feedingShooter = true;
    CommandScheduler.getInstance().cancel(moveIntakeUp);
    indexer.stop();
    hopper.setHopperRoller(0);
    intake.deployIntake();
    intake.stopWheels();
    timer.stop();
    timer.reset();
    System.out.println("FeedShooterStopped");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
