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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;
import frc.robot.LogInputs.LimelightInputs;
import frc.robot.LogInputs.QuestInputs;
import frc.robot.LogInputs.QuestInputsAutoLogged;
import frc.robot.settings.OdometryUpdatingState;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class Quest extends SubsystemBase {
  QuestNav questNav = new QuestNav();
  //for the transform3D below, x is forward, either y or z is how far left of center the quest is. The other one is upwardsness, but that doesn't matter to us. The Rotation3d matters for some reason, even though we don't get pitch, roll, or yaw from the quest
  Transform3d robotToQuest = new Transform3d(-0.278, -0.165, -0.165, new Rotation3d(0, 0, Math.toRadians(-180)));
  Matrix<N3, N1> questnavStandardDeviations = VecBuilder.fill(0.02, 0.02, 0.035); //The suggested Standerd Deviations for QuestNav
  DrivetrainSubsystem drivetrain;
  QuestInputsAutoLogged inputs;
  Limelight limelight;
  double lastFrameCount = 0;
  /** Creates a new Quest. */
  public Quest(DrivetrainSubsystem drivetrain) {
    this.drivetrain = drivetrain;
    limelight = Limelight.getInstance();
    inputs = new QuestInputsAutoLogged();
    questNav.onDisconnected(()->RobotState.getInstance().odometryUpdatingState = OdometryUpdatingState.drivetrainAndLimlights);
    questNav.onConnected(()->RobotState.getInstance().odometryUpdatingState = OdometryUpdatingState.Quest);
    questNav.onTrackingLost(()->RobotState.getInstance().odometryUpdatingState = OdometryUpdatingState.drivetrainAndLimlights);
    questNav.onTrackingAcquired(()->RobotState.getInstance().odometryUpdatingState = OdometryUpdatingState.Quest);
  }
  public void setQuestNavPose(Pose3d robotPose) {
    questNav.setPose(robotPose.transformBy(robotToQuest));
  }
  public void setQuestNavPose(Pose2d robotPose){
    setQuestNavPose(new Pose3d(robotPose.getX(), robotPose.getY(), 0, new Rotation3d(0, 0, robotPose.getRotation().getRadians())));
  }
  public void resetQuestPose(){
    if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue) {
      drivetrain.setGyroscope(180);
    } else {
      drivetrain.setGyroscope(0);
    }

    if(DriverStation.getAlliance() != null && DriverStation.getAlliance().get() == Alliance.Blue){
      setQuestNavPose(new Pose3d(new Translation3d(3.6,4.05,0), new Rotation3d(drivetrain.getOdometryRotation()))); 
    } else {
      setQuestNavPose(new Pose3d(new Translation3d(12.9,4.05,0), new Rotation3d(drivetrain.getOdometryRotation()))); 
    }
  }

  @Override
  public void periodic() {
    inputs.questFrames = questNav.getAllUnreadPoseFrames();
    inputs.frameCountPresent = questNav.getFrameCount().isPresent();
    inputs.frameCount = questNav.getFrameCount().orElse(0);
    inputs.isConnected = questNav.isConnected();
    inputs.isTracking = questNav.isTracking();
    inputs.batteryPercentage = questNav.getBatteryPercent().orElse(0);

    Logger.processInputs("Quest", inputs);

    RobotState.getInstance().questIsConnected = inputs.isConnected && inputs.isTracking && inputs.frameCount != lastFrameCount;
    lastFrameCount = inputs.frameCount;
    SmartDashboard.putBoolean("Quest Connected", RobotState.getInstance().questIsConnected);
    questNav.commandPeriodic();

    SmartDashboard.putString("OdometryUpdatingState", RobotState.getInstance().odometryUpdatingState.toString());
    if(RobotState.getInstance().odometryUpdatingState == OdometryUpdatingState.Quest && RobotState.getInstance().questIsConnected) {
      if (limelight.getTrustedPose()!= null) {
        Pair<Pose2d, LimelightInputs> estimate = limelight.getTrustedPose();
        if (drivetrain.getDrivetrainVelocity() < 0.2 && Math.abs(drivetrain.getAngularVelocity()) < 10 && estimate.getSecond().tagCount != 0 && drivetrain.isFlat()) {
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
          drivetrain.updateOdometryWithVision(new Pair<>(new Pose2d(robotPose.getX(), robotPose.getY(), robotPose.getRotation().toRotation2d()), timestamp));
        }
      }
    }else{
      if(limelight.getTrustedPose() != null && limelight.getTrustedPose().getSecond().tagCount != 0 && drivetrain.isFlat()) {
        drivetrain.updateOdometryWithVision(new Pair<Pose2d,Double>(limelight.getTrustedPose().getFirst(), limelight.getTrustedPose().getSecond().timeStampSeconds));
      }
    }
   }
 }

