// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.settings.Constants.DriveConstants.k_THETA_D;
import static frc.robot.settings.Constants.DriveConstants.k_THETA_I;
import static frc.robot.settings.Constants.DriveConstants.k_THETA_P;
import static frc.robot.settings.Constants.DriveConstants.k_XY_D;
import static frc.robot.settings.Constants.DriveConstants.k_XY_I;
import static frc.robot.settings.Constants.DriveConstants.k_XY_P;
import static frc.robot.settings.Constants.XboxDriver.X_AXIS;
import static frc.robot.settings.Constants.XboxDriver.Y_AXIS;
import static frc.robot.settings.Constants.XboxDriver.Z_AXIS;

import java.io.IOException;
import java.util.HashMap;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.Drive;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

import static frc.robot.settings.Constants.SubsystemsEnabled.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private DrivetrainSubsystem drivetrain;
  private Shooter shooter;
  private Climber climber;
  private Intake intake;
  private Indexer indexer;
  private Limelight limelight;
  private Drive defaultDriveCommand;
  private SendableChooser<Command> autoChooser;
  private final XboxController driveController;

  DoubleSupplier ControllerForwardAxisSupplier;
  DoubleSupplier ControllerSidewaysAxisSupplier;
  DoubleSupplier ControllerZAxisSupplier;
  BooleanSupplier ZeroGyroSup;

  public static HashMap<String, Command> eventMap;

  public RobotContainer() {
    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    driveController = new XboxController(0);
    autoChooser = new SendableChooser<>();
    eventMap = new HashMap<>();

    // Drive controls
    ControllerSidewaysAxisSupplier = () -> modifyAxis(-driveController.getRawAxis(X_AXIS), 0);
    ControllerForwardAxisSupplier = () -> modifyAxis(-driveController.getRawAxis(Y_AXIS), 0);
    ControllerZAxisSupplier = () -> modifyAxis(-driveController.getRawAxis(Z_AXIS), 0);
    ZeroGyroSup = driveController::getStartButton;

    if (DRIVE_TRAIN_EXISTS) {
      driveTrainInit();
      configureDriveTrain();
    }

    if (LIMELIGHTS_EXIST) {
      limelightInit();
    }

    if (SHOOTER_EXISTS) {
      shooterInit();
    }

    if (CLIMBER_EXISTS) {
      climberInit();
    }

    if (INDEXER_EXISTS) {
      indexerInit();
    }

    if (INTAKE_EXISTS) {
      intakeInit();
    }

    SmartDashboard.putBoolean("use limelight", false);
    SmartDashboard.putBoolean("trust limelight", false);
    configureBindings();
  }

  private void driveTrainInit() {
    drivetrain = new DrivetrainSubsystem();

    defaultDriveCommand = new Drive(
        drivetrain,
        () -> false,
        ControllerForwardAxisSupplier,
        ControllerSidewaysAxisSupplier,
        ControllerZAxisSupplier);
    drivetrain.setDefaultCommand(defaultDriveCommand);
  }

  private void configureDriveTrain() {
    try {
      AutoBuilder.configure(
          drivetrain::getPose, // Pose2d supplier
          drivetrain::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
          drivetrain::getChassisSpeeds,
          (speeds) -> drivetrain.drive(speeds),
          new PPHolonomicDriveController(
              new com.pathplanner.lib.config.PIDConstants(
                  k_XY_P, k_XY_I,
                  k_XY_D), // PID constants to correct for translation error (used to create the X
              // and Y PID controllers)
              new com.pathplanner.lib.config.PIDConstants(
                  k_THETA_P, k_THETA_I,
                  k_THETA_D) // PID constants to correct for rotation error (used to create the
          // rotation controller)
          ),
          RobotConfig.fromGUISettings(),
          () -> DriverStation.getAlliance().get().equals(Alliance.Red),
          drivetrain);
    } catch (org.json.simple.parser.ParseException a) {
      System.out.println("got ParseException trying to configure AutoBuilder");
    } catch (IOException b) {
      System.out.println("got IOException thrown trying to configure autobuilder " + b.getMessage());
    }
  }

  private void limelightInit() {
    limelight = Limelight.getInstance();
  }

  private void shooterInit() {
    shooter = new Shooter();
  }

  private void intakeInit() {
    intake = new Intake();
  }

  private void climberInit() {
    climber = new Climber();
  }
  
  private void indexerInit() {
    indexer = new Indexer();
  }

  private void autoInit() {
     DrivetrainExists = true; //Preferences.getBoolean("DrivetrainExists", true);
    LimelightExists = Preferences.getBoolean("Use Limelight", true);


  }

  /**
   * Takes both axis of a joystick, returns an angle from -180 to 180 degrees, or
   * {@link Constants.PS4Driver.NO_INPUT} (double = 404.0) if the joystick is at
   * rest position
   */

  /** Takes both axis of a joystick, returns a double from 0-1 */

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4DriverController
   * PS4Driver} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    if (DRIVE_TRAIN_EXISTS) {
      SmartDashboard.putData("drivetrain", drivetrain);
      new Trigger(ZeroGyroSup).onTrue(new InstantCommand(drivetrain::zeroGyroscope));

      InstantCommand setOffsets = new InstantCommand(drivetrain::setEncoderOffsets) {
        public boolean runsWhenDisabled() {
          return true;
        };
      };
      InstantCommand zeroGyroscope = new InstantCommand(drivetrain::zeroGyroscope) {
        public boolean runsWhenDisabled() {
          return true;
        };
      };

      SmartDashboard.putData("zeroGyroscope", zeroGyroscope);
      SmartDashboard.putData("set offsets", setOffsets);
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  private double modifyAxis(double value, double deadband) {
    // Deadband
    value = MathUtil.applyDeadband(value, deadband);
    // Square the axis
    value = Math.copySign(value * value, value);
    return value;
  }

  public void robotInit() {
    drivetrain.zeroGyroscope();
  }

  public void teleopInit() {
  }

  public void teleopPeriodic() {
    SmartDashboard.putNumber("RobotAngle", drivetrain.getGyroscopeRotation().getDegrees());
    SmartDashboard.putNumber("GetPose", drivetrain.getPose().getRotation().getDegrees());
  }
}
