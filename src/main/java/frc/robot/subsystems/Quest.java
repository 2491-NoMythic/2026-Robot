// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;
import frc.robot.LogInputs.LimelightInputs;
import frc.robot.LogInputs.QuestInputs;
import frc.robot.LogInputs.QuestInputsAutoLogged;
import frc.robot.subsystems.DrivetrainSubsystem;
public class Quest extends SubsystemBase {
  QuestNav questNav = new QuestNav();
  Transform3d robotToQuest =new Transform3d();
  Matrix<N3, N1> questnavStandardDeviations = VecBuilder.fill(0.02, 0.02, 0.035); //The suggested Standerd Deviations for QuestNav
  DrivetrainSubsystem drivetrain;
  QuestInputsAutoLogged inputs;
  Limelight limelight;
  /** Creates a new Quest. */
  public Quest(DrivetrainSubsystem drivetrain) {
    this.drivetrain = drivetrain;
    limelight = Limelight.getInstance();
    inputs = new QuestInputsAutoLogged();
  }
  public void setQuestNavPose(Pose3d robotPose) {
    questNav.setPose(robotPose.transformBy(robotToQuest));
  }
  public void setQuestNavPose(Pose2d robotPose){
    setQuestNavPose(new Pose3d(robotPose.getX(), 0, robotPose.getY(), new Rotation3d(0, 0, robotPose.getRotation().getRadians())));
  }
  public void resetQuestPose(){
    drivetrain.zeroGyroscope();
    if(DriverStation.getAlliance() != null && DriverStation.getAlliance().get() == Alliance.Blue){
      setQuestNavPose(new Pose3d(new Translation3d(3.6,0,4.05),new Rotation3d(drivetrain.getOdometryRotation()))); 
    }else {
      setQuestNavPose(new Pose3d(new Translation3d(12.9,0,4.05),new Rotation3d(drivetrain.getOdometryRotation()))); 
    }
  }

  @Override
  public void periodic() {
    inputs.questFrames = questNav.getAllUnreadPoseFrames();
    Logger.processInputs("Quest", inputs);
    questNav.commandPeriodic();
    if (limelight.getTrustedPose()!= null){
      Pair<Pose2d, LimelightInputs> estimate = limelight.getTrustedPose();
      if (drivetrain.getDrivetrainVelocity() < 0.2 && Math.abs(drivetrain.getAngularVelocity()) < 720 && estimate.getSecond().tagCount != 0 ) {
        setQuestNavPose(estimate.getFirst());
      }
    }
    PoseFrame[] questFrames = inputs.questFrames;
    for (PoseFrame questFrame : questFrames) {
      if (questFrame.isTracking()) {
        // Get the pose of the Quest
        Pose3d questPose = questFrame.questPose3d();
        // Get timestamp for when the data was sent
        double timestamp = questFrame.dataTimestamp();
        // Transform by the mount pose to get your robot pose
        Pose3d robotPose = questPose.transformBy(robotToQuest.inverse());
        // addVisionMeasurement not working but this is what it said in docs
        drivetrain.updateOdometryWithVision(new Pair<>(new Pose2d(robotPose.getX(), robotPose.getZ(), robotPose.getRotation().toRotation2d()), timestamp));
      }
    }
   }
 }

