// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.DrivetrainSubsystem; 
  //Imports the DrivetrainSubsystem class so we can use it in this Command
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Shooter;

import frc.robot.subsystems.Shooter; //duplicate
import frc.robot.subsystems.RobotState; //unused

public class CommandName extends Command { 
    //Defines CommandName as a CommandName class that inherits a Command (has a Command's features)
  DrivetrainSubsystem drivetrain; 
    //CommandNames will have a DrivetrainSubsystem
  Hopper hopper; 
  Shooter shooter; 

  boolean shooterReady; //booleans are either true  or false
  int driveSpeed; //int(egers) are whole numbers

  //The Constructor \/
  public CommandName(DrivetrainSubsystem drivetrain, Hopper hopper, Shooter shooter) {  
      //When CommandName is called, it will need to be given these variables  
    this.drivetrain = drivetrain; 
      //The initial value of drivetrain is the drivetrain CommandName is given
    this.hopper = hopper; 
    this.shooter = shooter; 
    driveSpeed = 5;
      //The initial value of driveSpeed is 5
    addRequirements(drivetrain); //Tells CommandName not to run alongside other Commands using these subsystems
  }

  @Override
  public void initialize() {
      //Command feature, called when a command starts running
    shooter.shooterOn();
  } 

  @Override
  public void execute() {  
      //Command feature, Called periodically while a command runs
    drivetrain.pointWheelsInward();
      //Calls the pointWheelsInward function from the DrivetrainSubsystem
    drivetrain.stop();
  }
  
  @Override 
  public void end(boolean interrupted) { 
      //Command feature, Called once the command ends or is interrupted.
    drivetrain.stop();
  }

  @Override
  public boolean isFinished() { 
      //Command feature, Returns true when the command should end.
    return false;
  }
}
