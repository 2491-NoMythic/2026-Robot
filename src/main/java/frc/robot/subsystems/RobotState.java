package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.settings.ClimberState;
import frc.robot.settings.HopperState;

import java.util.ArrayList;
import java.util.List;
import frc.robot.helpers.TimerPhase;

public class RobotState {
  private static RobotState instance;
  public boolean Aimed;
  public boolean LimelightsUpdated;
  public boolean lightsReset;
  public double odometerOrientation;
  public Pose2d robotPosition = new Pose2d();
  public boolean intakeRunning;
  public boolean linedUpToShoot;
  public boolean indexerRunning;
  public boolean shooting;
  public boolean hopperExpandedVertically;
  public boolean hopperExpandedHorizontally;
  public ClimberState climberState;
  public HopperState hopperState;
  public boolean halfFullSwitchTriggered;
  public boolean fullSwitchTriggered;

  public double aimingPitch;
  public double aimingYaw;

  public static List<TimerPhase> timerPhases;
  public static TimerPhase currentPhase;
  public static TimerPhase autoPhase;
  public static TimerPhase nullPhase;

  public static int matchTime;

  public RobotState() {
    // sets any values that aren't periodically updated by a subsystem to a value,
    // so that they won't return null if called before they are updated
    initializeTimerPhases();
  }

  public static void initializeTimerPhases(){
    timerPhases = new ArrayList<TimerPhase>();
    
    timerPhases.add(new TimerPhase(140, 10, "TRANSITION"));
    timerPhases.add(new TimerPhase(130, 25, "SHIFT 1"));
    timerPhases.add(new TimerPhase(105, 25, "SHIFT 2"));
    timerPhases.add(new TimerPhase(80, 25, "SHIFT 3"));
    timerPhases.add(new TimerPhase(55, 25, "SHIFT 4"));
    timerPhases.add(new TimerPhase(30, 30, "ENDGAME"));

    autoPhase = new TimerPhase(20, 20, "AUTO");
    nullPhase = new TimerPhase(2491, 2491, "NULL");
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

  /* public static int getMatchTimeCountingUp(){ 
    if (DriverStation.isAutonomousEnabled()) {
      return ((int) DriverStation.getMatchTime());
    }
    else if (DriverStation.isTeleopEnabled()) {
      return 140 - (int) DriverStation.getMatchTime();
    } else {
      return 2491;
    }
  } */

  public static int getMatchTime(){
    return (int)DriverStation.getMatchTime();
  }

  public static void updatePhase(){
    matchTime = (int) DriverStation.getMatchTime();

    if(DriverStation.isAutonomousEnabled()){
      currentPhase = autoPhase;
    } else if (DriverStation.isTeleopEnabled()) {
      for (TimerPhase phase : timerPhases) {
        if(phase.isCurrentPhase(matchTime)) {
          currentPhase = phase;
          return;
        }
      }
    } else {
      currentPhase = nullPhase;
    }
  }

  public static String getPhase(){
    updatePhase();
    return currentPhase.getPhaseName();
  }

  public static int getPhaseTimeLeft(){
    updatePhase();
    return currentPhase.getPhaseTimeLeft(matchTime);
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
