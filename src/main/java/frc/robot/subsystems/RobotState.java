package frc.robot.subsystems;

import java.util.Optional;

import javax.print.attribute.standard.MediaSize.Other;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotState {
  private static RobotState instance;
  public boolean LimelightsUpdated;
  public boolean lightsReset;
  public double odometerOrientation;
  public Pose2d robotPosition = new Pose2d();

  public double aimingPitch;
  public double aimingYaw;

  public RobotState() {
    // sets any values that aren't periodically updated by a subsystem to a value,
    // so that they won't return null if called before they are updated
  }

  public static boolean IsAlliance(Alliance alliance) {
    Optional<Alliance> Current = DriverStation.getAlliance();
    if (Current.isPresent()) {
      return Current.get() == alliance;
    } else {
      return false;
    }
  }

  public static RobotState getInstance() {
    if (instance == null) {
      instance = new RobotState();
    }
    return instance;
  }
  /**
   * @return true when the coralEndEffector or funnelIntake detects a coral
   */
}
