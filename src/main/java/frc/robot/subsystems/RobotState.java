package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.settings.ClimberState;

public class RobotState {
  private static RobotState instance;
  public boolean LimelightsUpdated;
  public boolean lightsReset;
  public double odometerOrientation;
  public Pose2d robotPosition = new Pose2d();
  public int hopperFullness;
  public boolean intakeRunning;
  public boolean linedUpToShoot;
  public boolean indexerRunning;
  public boolean hopperExpandedVertically;
  public boolean hopperExpandedHorizontally;
  public ClimberState climberState;

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
  
  public static boolean isSecond() {
    String gameData;
    gameData = DriverStation.getGameSpecificMessage();
    if(gameData.length() > 0)
    {
      switch (gameData.charAt(0))
      {
       case 'B' :
         return IsAlliance(Alliance.Blue);
       case 'R' :
         return IsAlliance(Alliance.Red);
        default :
         return false;
     }
    } else {
      return false;
    }
  }
  
  public static String getPhase() {
    double matchTime = DriverStation.getMatchTime();
    if (DriverStation.isAutonomousEnabled()) {
      return "AUTONOMOUS";
    }
    else if (DriverStation.isTeleopEnabled()) {
      if (matchTime <= 140 && matchTime > 130) {
        return "TRANSITION SHIFT";
      } else if (matchTime <= 130 && matchTime > 105) {
        return "SHIFT 1";
      } else if (matchTime <= 105 && matchTime > 80) {
        return "SHIFT 2";
      } else if (matchTime <= 80 && matchTime > 55) {
        return "SHIFT 3";
      } else if (matchTime <= 55 && matchTime > 30) {
        return "SHIFT 4";
      } else if (matchTime <= 30 && matchTime > 0) {
        return "END GAME";
      } else {
        return "NOT FOUND";
      }
    }
    else {
      return "NO GAME";
    }
  }

  public static int getPhaseTime() {
    double matchTime = DriverStation.getMatchTime();
    String phase = getPhase();
    if (DriverStation.isAutonomousEnabled()) {
        return (int) matchTime;
    } 
    else if (DriverStation.isTeleopEnabled()) {
      if (phase == "TRANSITION SHIFT") {
        return (int) matchTime - 130;
      } else if (phase == "PHASE 1") {
        return (int) matchTime - 105;
      } else if (phase == "PHASE 2") {
        return (int) matchTime - 80;
      } else if (phase == "PHASE 3") {
        return (int) matchTime - 55;
      } else if (phase == "PHASE 4") {
        return (int) matchTime - 30;
      } else if (phase == "END GAME") {
        return (int) matchTime - 0;
      } else {
        return (int) matchTime;
      }
    }
    else {
      return 2491;
    }
  }

  public static Boolean hubActive() {
    if (isSecond()) {
      if (getPhase() == "SHIFT 1") {
        return false;
      } else if (getPhase() == "SHIFT 3") {
        return false;
      } else {
        return true;
      }
    } else {
      if (getPhase() == "SHIFT 2") {
        return false;
      } else if (getPhase() == "SHIFT 4") {
        return false;
      } else {
        return true;
      }
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
