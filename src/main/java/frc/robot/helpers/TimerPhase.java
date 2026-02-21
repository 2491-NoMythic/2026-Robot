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
  
  public int getPhaseTime(int totalTime){
    return totalTime - startTime;
  }

  public int getPhaseTimeLeft(int totalTime){
    return length - (totalTime - startTime);
  }

  public boolean isCurrentPhase(int totalTime){
    return (startTime + length) > totalTime && startTime <= totalTime;
  }

  public String getPhaseName() {
    return name;
  }
}