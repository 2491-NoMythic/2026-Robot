// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.settings.Constants.QuestNavConstants;
public class Quest extends SubsystemBase {
  QuestNav questNav = new QuestNav();
  Transform3d robotToQuest =new Transform3d();
  DrivetrainSubsystem drivetrain;
  Matrix<N3, N1> questnavStandardDeviations = VecBuilder.fill(0.02, 0.02, 0.035); //The suggested Standerd Deviations for QuestNav
  /** Creates a new Quest. */
  public Quest(DrivetrainSubsystem drivetrain) {
    this.drivetrain = drivetrain;
  }
  public void setQuestNavPose(Pose3d pose) {
    questNav.setPose(pose.transformBy(robotToQuest));
  }
  public void updateVisionMeasurement(){
    PoseFrame[] questFrames = questNav.getAllUnreadPoseFrames();
    for (PoseFrame questFrame : questFrames) {
      if (questNav.isConnected() && questNav.isTracking()) {
        if (questFrame.isTracking()) {
          // Get the pose of the Quest
          Pose3d questPose = questFrame.questPose3d();
          // Get timestamp for when the data was sent
          double timestamp = questFrame.dataTimestamp();
          // Transform by the mount pose to get your robot pose
          Pose3d robotPose = questPose.transformBy(robotToQuest.inverse());
          // Add the measurement to our estimator
          drivetrain.addVisionMeasurement(robotPose, timestamp, questnavStandardDeviations);
        }
      }
    }
  }

  @Override
  public void periodic() {
    questNav.commandPeriodic();
    updateVisionMeasurement();
  }
}
