// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.settings.Constants.Vision;
import frc.robot.LogInputs.LimelightDetectorInputsAutoLogged;
import frc.robot.settings.LimelightDetectorData;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Limelight;

public class CollectFuel extends Command {

  DrivetrainSubsystem drivetrain;
  Limelight limelight;
  double runsInvalid;

  PIDController txController;
  PIDController tyController;
  SlewRateLimiter tyLimiter;
  Boolean closeFuel;
  double tx;
  double ty;
  double ta;
  /** Creates a new CollectFuel. */
  public CollectFuel(DrivetrainSubsystem drivetrain) {
    addRequirements(drivetrain);
    this.drivetrain = drivetrain;
    this.limelight = Limelight.getInstance();
    
    txController = new PIDController(
      0.06,
      0,
      0);
    tyController = new PIDController(
      0.06,
      0,
      0);
    tyLimiter = new SlewRateLimiter(20, -20, 0);
    txController.setSetpoint(0);
    tyController.setSetpoint(0);
    txController.setTolerance(3.5);
    tyController.setTolerance(2.5);
  }
  
  @Override
  public void initialize() {
    SmartDashboard.putBoolean("CollectFuel/comandrunning", true);
    runsInvalid = 0;
    closeFuel = false;
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    LimelightDetectorInputsAutoLogged detectorData = limelight.getDetectorData();
    tx = detectorData.tx;
    ty = detectorData.ty;
    ta = detectorData.ta;
    
    if(detectorData.ta != 0){
        double forwardSpeed = tyLimiter.calculate(tyController.calculate(-ty));
        forwardSpeed = forwardSpeed > 1 ? 1 : forwardSpeed;

        double sidewaysSpeed = txController.calculate(tx);
        sidewaysSpeed = sidewaysSpeed > 1 ? 1 : sidewaysSpeed;
        sidewaysSpeed = sidewaysSpeed < -1 ? -1 : sidewaysSpeed;
        
        drivetrain.drive(new ChassisSpeeds(
          forwardSpeed,
          sidewaysSpeed,
          0));

        SmartDashboard.putNumber("CollectFuel/forward speed limited", forwardSpeed);
        SmartDashboard.putNumber("CollectFuel/sideways speed limited", sidewaysSpeed);
    } 
    else {
      drivetrain.drive(new ChassisSpeeds(
        0, 0, 0));
        runsInvalid++;
    }
    
    SmartDashboard.putNumber("CollectFuel/calculated sideways meters per second", txController.calculate(tx));
    SmartDashboard.putNumber("CollectFuel/calculated forward meters per second", tyController.calculate(ty));
    SmartDashboard.putNumber("DETECTOR/tx", tx);
    SmartDashboard.putNumber("DETECTOR/ty", ty);
    SmartDashboard.putNumber("DETECTOR/ta", ta);
    SmartDashboard.putBoolean("CollectFuel/isFuelSeen", detectorData.ta != 0);
    SmartDashboard.putNumber("CollectFuel/runsInvalid", runsInvalid);
    // drives the robot forward faster if the object is higher up on the screen, and turns it more based on how far away the object is from x=0
  }
  


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    runsInvalid = 0;
    SmartDashboard.putBoolean("CollectFuel/comandrunning", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ((tyController.atSetpoint() && txController.atSetpoint()) || runsInvalid>5); 
  }
}