package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.settings.OdometryUpdatingState;

public class RobotState {
  private static RobotState instance;
  public boolean Aimed;
  public boolean lightsReset;
  public Pose2d robotPosition = new Pose2d();
  public boolean intakeRunning;
  public boolean indexerRunning;
  public boolean shooting;
  public boolean feedingShooter;
  public OdometryUpdatingState odometryUpdatingState;

  public double aimingPitch;
  public double aimingYaw;
  public double desiredShooterSpeed;
  public boolean overrideShooterSpeed;

  public boolean lightsRobotDisabled;
  public boolean lightsShooterOutOfRange;
  public boolean lightsIndexing;
  public double lightsRobotSpeed;
  public boolean lightsMatchPlayed;


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
}
