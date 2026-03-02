package frc.robot.Commands;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public class DriveConstantSpeed extends Command{
    Timer timer;
    DrivetrainSubsystem drivetrain;

    double speed;
    double time;
    
  public DriveConstantSpeed(DrivetrainSubsystem drivetrain, double speed, double time) {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);

    this.speed = speed;
    this.time = time;

    timer = new Timer();
  }
    @Override
    public void initialize() {
      drivetrain.drive(new ChassisSpeeds(speed, 0, 0));
      timer.reset();
      timer.start();
    }

    @Override
  public void execute() {}

    @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
  }

    @Override
  public boolean isFinished(){
    return timer.get() > time;
  }
}
