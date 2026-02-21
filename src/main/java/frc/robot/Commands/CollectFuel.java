// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.Arrays;
import java.util.List;

import com.revrobotics.ColorSensorV3.RawColor;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.settings.Constants.Vision;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawDetection;
import frc.robot.LogInputs.LimelightDetectorInputsAutoLogged;
import frc.robot.settings.LimelightDetectorData;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Limelight;

public class CollectFuel extends Command {

  DrivetrainSubsystem drivetrain;
  Limelight limelight;
  double runsInvalid;

  boolean inAuto;
  Timer autoTimer;
  float timeLimit;
  Pose2d autoEndPose;

  PIDController txController;
  PIDController tyController;
  SlewRateLimiter tyLimiter;
  Boolean closeFuel;
  double tx;
  double ty;
  double ta;

  /** Creates a new CollectFuel. */
  public CollectFuel(DrivetrainSubsystem drivetrain, boolean inAuto, Pose2d autoEndPose) {
    addRequirements(drivetrain);
    this.drivetrain = drivetrain;
    this.limelight = Limelight.getInstance();
    this.inAuto = inAuto;
    this.autoEndPose = autoEndPose;

    this.timeLimit = 5;
    
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

    autoTimer.start();
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!inAuto || !autoTimer.hasElapsed(timeLimit)){
      LimelightDetectorInputsAutoLogged detectorData = limelight.getDetectorData();
      
      RawDetection[] rawDetections = LimelightHelpers.getRawDetections(limelight.detectorLimelight.limelightName);

      RawDetection[] leftDetections = Arrays.stream(rawDetections).filter(element -> element.txnc < 0).toArray(RawDetection[]::new);
      RawDetection[] rightDetections = Arrays.stream(rawDetections).filter(element -> element.txnc >= 0).toArray(RawDetection[]::new);
      
      RawDetection[] largerSide;
      RawDetection[] smallerSide;
      if(leftDetections.length > rightDetections.length){
        largerSide = leftDetections;
        smallerSide = rightDetections;
      } else {
        largerSide = rightDetections;
        smallerSide = leftDetections;
      }

      float largerSideXAverage = 0;
      for (RawDetection detec : largerSide) largerSideXAverage += detec.txnc;
      largerSideXAverage = largerSideXAverage/smallerSide.length;

      float smallerSideXAverage = 0;
      for (RawDetection detec : smallerSide) largerSideXAverage += detec.txnc;
      smallerSideXAverage = smallerSideXAverage/smallerSide.length;

      float weightedSideAverage = largerSideXAverage;
      
      //tx = detectorData.tx;
      tx = weightedSideAverage;
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
      
      SmartDashboard.putNumber("DETECTOR/tx", tx);
      SmartDashboard.putNumber("DETECTOR/ty", ty);
      SmartDashboard.putNumber("DETECTOR/ta", ta);
      SmartDashboard.putBoolean("CollectFuel/isFuelSeen", detectorData.ta != 0);
      SmartDashboard.putNumber("CollectFuel/runsInvalid", runsInvalid);
      // drives the robot forward faster if the object is higher up on the screen, and turns it more based on how far away the object is from x=0
    } else {
      drivetrain.moveTowardsPose(autoEndPose);
    }
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
    Transform2d diffPose = drivetrain.getPose().minus(autoEndPose);
    return ((tyController.atSetpoint() && txController.atSetpoint()) || runsInvalid>5 || (Math.abs(diffPose.getX()) <= 0.25 && Math.abs(diffPose.getY()) <= 0.25)); 
  }
}