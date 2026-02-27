package frc.robot.helpers;

public class TimerPhase{
  int startTime;
  int length;
  String name;

  public TimerPhase(int startTime, int length, String name){
    this.startTime = startTime;
    this.length = length;
    this.name = name;
  }

  public int getPhaseTimeLeft(int totalTimeLeft){
    return totalTimeLeft - (startTime - length);
  }

  public boolean isCurrentPhase(int totalTime){
    return (startTime - length) < totalTime && startTime >= totalTime;
  }

  public String getPhaseName() {
    return name;
  }
}