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
import static frc.robot.settings.Constants.XboxDriver.DEADBAND_NORMAL;
import static frc.robot.settings.Constants.XboxDriver.DRIVE_CONTROLLER_ID;
import static frc.robot.settings.Constants.XboxDriver.OPERATOR_CONTROLLER_ID;
import static frc.robot.settings.Constants.XboxDriver.X_AXIS;
import static frc.robot.settings.Constants.XboxDriver.Y_AXIS;
import static frc.robot.settings.Constants.XboxDriver.Z_AXIS;

import java.io.IOException;
import java.util.HashMap;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
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
import frc.robot.Commands.AimHood;
import frc.robot.Commands.AimRobotMoving;
import frc.robot.Commands.ClimbDown;
import frc.robot.Commands.ClimbUp;
import frc.robot.Commands.Drive;
import frc.robot.Commands.RunIndexer;
import frc.robot.Commands.RunShooterVelocity;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

import static frc.robot.settings.Constants.SubsystemsEnabled.*;
import frc.robot.Commands.AimRobotMoving;

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
  private final XboxController operatorController;

  DoubleSupplier ControllerForwardAxisSupplier;
  DoubleSupplier ControllerSidewaysAxisSupplier;
  DoubleSupplier ControllerZAxisSupplier;
  BooleanSupplier ZeroGyroSup;
  BooleanSupplier AimRobotMovingSup;
  BooleanSupplier ClimberUpSup;
  BooleanSupplier ClimberDownSup;
  BooleanSupplier ShooterToggleSupplier;
  BooleanSupplier HoodUpSupplier;
  BooleanSupplier HoodDownSupplier;
  BooleanSupplier IndexerSup;
  boolean manualShooterOn = false;

  public static HashMap<String, Command> eventMap;

  public RobotContainer() {

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    driveController = new XboxController(DRIVE_CONTROLLER_ID);
    operatorController = new XboxController(OPERATOR_CONTROLLER_ID);
    autoChooser = new SendableChooser<>();
    eventMap = new HashMap<>();

    // Drive controls
    ControllerSidewaysAxisSupplier = () -> modifyAxis(-driveController.getRawAxis(X_AXIS), 0);
    ControllerForwardAxisSupplier = () -> modifyAxis(-driveController.getRawAxis(Y_AXIS), 0);
    ControllerZAxisSupplier = () -> modifyAxis(-driveController.getRawAxis(Z_AXIS), 0);
    ZeroGyroSup = driveController::getStartButton;
    AimRobotMovingSup = ()-> driveController.getLeftTriggerAxis() >= 0.5;
    //Shooter controls
    HoodUpSupplier = () -> operatorController.getLeftY() < -0.5;
    HoodDownSupplier = () -> operatorController.getLeftY() > 0.5;
    ShooterToggleSupplier = operatorController::getXButton;
    IndexerSup = ()-> operatorController.getLeftTriggerAxis() > 0.5;
    //climber controls
    ClimberDownSup = ()-> operatorController.getRightY() > 0.5;
    ClimberUpSup = ()-> operatorController.getRightY() < -0.5;

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
    autoInit();
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
    shooter.setDefaultCommand(new AimHood(shooter));
    new Trigger(HoodUpSupplier).whileTrue(new InstantCommand(()->shooter.setHoodMotor(0.2), shooter)).onFalse(new InstantCommand(()->shooter.setHoodMotor(0), shooter));
    new Trigger(HoodDownSupplier).whileTrue(new InstantCommand(()->shooter.setHoodMotor(-0.2), shooter)).onFalse(new InstantCommand(()->shooter.setHoodMotor(0), shooter));
    new Trigger(ShooterToggleSupplier).onTrue(new InstantCommand(()->manualShooterOn = !manualShooterOn));
    new Trigger(()->manualShooterOn).onTrue(new InstantCommand(()->shooter.set(0.2), shooter)).onFalse(new InstantCommand(()->shooter.stop(), shooter));
  }

  private void intakeInit() {
    intake = new Intake();
  }

  private void climberInit() {
    climber = new Climber();

    new Trigger(ClimberDownSup).whileTrue(new InstantCommand(()->climber.climberDown())).onFalse(new InstantCommand(()->climber.stop()));
    new Trigger(ClimberUpSup).whileTrue(new InstantCommand(()->climber.climberUp())).onFalse(new InstantCommand(()->climber.stop()));
  }
  
  private void indexerInit() {
    indexer = new Indexer();
  }

  private void autoInit() {
    configureDriveTrain();
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
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
    if(DRIVE_TRAIN_EXISTS){
       new Trigger(AimRobotMovingSup).whileTrue(new AimRobotMoving(
        drivetrain,
        () -> modifyAxis(-driveController.getRawAxis(Y_AXIS), DEADBAND_NORMAL),
        () -> modifyAxis(-driveController.getRawAxis(X_AXIS), DEADBAND_NORMAL)
        ));
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

  void registerNamedCommands(){
    NamedCommands.registerCommand("ClimbUp", new ClimbUp());
    NamedCommands.registerCommand("ClimbDown", new ClimbDown());
    NamedCommands.registerCommand("RunIndexer", new RunIndexer(indexer, Z_AXIS));
    NamedCommands.registerCommand("ShooterVelocity", new RunShooterVelocity(shooter, Z_AXIS));
    NamedCommands.registerCommand("AimRobotMoving", new AimRobotMoving(drivetrain, ControllerSidewaysAxisSupplier, ControllerForwardAxisSupplier));
  }
}
