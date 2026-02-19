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
import static frc.robot.settings.Constants.HopperConstants.HOPPER_ROLLER_SPEED;
import static frc.robot.settings.Constants.SubsystemsEnabled.CLIMBER_EXISTS;
import static frc.robot.settings.Constants.SubsystemsEnabled.DRIVE_TRAIN_EXISTS;
import static frc.robot.settings.Constants.SubsystemsEnabled.INDEXER_EXISTS;
import static frc.robot.settings.Constants.SubsystemsEnabled.INTAKE_EXISTS;
import static frc.robot.settings.Constants.SubsystemsEnabled.LIGHTS_EXIST;
import static frc.robot.settings.Constants.SubsystemsEnabled.LIMELIGHTS_EXIST;
import static frc.robot.settings.Constants.SubsystemsEnabled.SHOOTER_EXISTS;
import static frc.robot.settings.Constants.XboxDriver.DRIVE_CONTROLLER_ID;
import static frc.robot.settings.Constants.XboxDriver.OPERATOR_CONTROLLER_ID;
import static frc.robot.settings.Constants.XboxDriver.X_AXIS;
import static frc.robot.settings.Constants.XboxDriver.Y_AXIS;
import static frc.robot.settings.Constants.XboxDriver.Z_AXIS;

import java.io.IOException;
import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.AimAtHub;
import frc.robot.Commands.AimAtLocation;
import frc.robot.Commands.AimHood;
import frc.robot.Commands.AimRobot;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import frc.robot.Commands.AutomaticClimb;
import frc.robot.Commands.ClimberArmDown;
import frc.robot.Commands.ClimberArmUp;
import frc.robot.Commands.CollectFuel;
import frc.robot.Commands.Drive;
import frc.robot.Commands.MoveToClimbingPose;
import frc.robot.Commands.Outtake;
import frc.robot.Commands.OverBump;
import frc.robot.Commands.FeedShooter;
import frc.robot.Commands.LightsCommand;
import frc.robot.Commands.MoveToClimbingPose;
import frc.robot.Commands.LockYAxisForCrossing;
import frc.robot.Commands.Outtake;
import frc.robot.Commands.RunIntake;
import frc.robot.Commands.RunShooterVelocity;
import frc.robot.Commands.AimAtLocation.Location;
import frc.robot.settings.Constants.IndexerConstants;
import frc.robot.settings.LightsEnums;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.RobotState;
import frc.robot.subsystems.Shooter;

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
  private Hopper hopper;
  private Limelight limelight;
  private Lights lights;
  private Drive defaultDriveCommand;
  private SendableChooser<Command> autoChooser;
  private final XboxController driveController;
  private final XboxController operatorController;
  private Timer autoTimer;

  private AimAtHub aimAtHub;
  private AimHood aimHood;

  DoubleSupplier ControllerForwardAxisSupplier;
  DoubleSupplier ControllerSidewaysAxisSupplier;
  DoubleSupplier ControllerZAxisSupplier;
  BooleanSupplier ZeroGyroSup;
  BooleanSupplier AimRobotMovingSup;
  BooleanSupplier TrenchAllignSup;
  BooleanSupplier BumpAllignSup;
  BooleanSupplier ClimberUpSup;
  BooleanSupplier ClimberDownSup;
  BooleanSupplier RetractIntakeSup;
  BooleanSupplier DeployIntakeSup;
  BooleanSupplier AutoIntakeSup;
  BooleanSupplier IntakeWheelSup;
  BooleanSupplier ShooterToggleSupplier;
  BooleanSupplier HoodUpSupplier;
  BooleanSupplier HoodDownSupplier;
  BooleanSupplier IndexerSup;
  BooleanSupplier AutoAimSupplier;
  BooleanSupplier ShootIfAimedSup;
  BooleanSupplier ForceHoodDownSupplier;
  BooleanSupplier crossBumpTowardsAllianceSup;
  boolean manualShooterOn = false;
  BooleanSupplier ManualHubShotSup;
  BooleanSupplier ManualTowerShotSup;
  BooleanSupplier ManualLeftTrenchShotSup;
  BooleanSupplier ManualRightTrenchShotSup;



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
    AutoAimSupplier = () -> driveController.getLeftTriggerAxis() >= 0.5;
    AutoIntakeSup = driveController::getXButton;
    //Shooter controls
    HoodUpSupplier = () -> operatorController.getLeftY() < -0.5;
    HoodDownSupplier = () -> operatorController.getLeftY() > 0.5;
    ShooterToggleSupplier = operatorController::getXButton;
    IndexerSup = ()-> driveController.getRightTriggerAxis() > 0.5;
    ForceHoodDownSupplier = driveController::getBackButton;
    ManualHubShotSup = operatorController::getBButton;
    ManualTowerShotSup = operatorController::getLeftStickButton;
    ManualLeftTrenchShotSup = operatorController::getRightStickButton;
    ManualRightTrenchShotSup = operatorController::getRightBumperButton;
    //Shooting Command is Right Trigger on drive controller. 
    //climber controls
    ClimberDownSup = operatorController::getAButton;
    //Climber Down is A button on operator controller
    ClimberUpSup = operatorController::getYButton;
    //Climber Up is Y button on operator controller
    //intake controls
    //RetractIntakeSup = driveController::getLeftStickButton;
    //DeployIntakeSup = driveController::getRightStickButton;
    IntakeWheelSup = driveController::getLeftBumperButton;
    //Trench Controls
    TrenchAllignSup = driveController::getBButton;
    BumpAllignSup = driveController::getRightStickButton;

    crossBumpTowardsAllianceSup = driveController::getYButton;

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

    if (LIGHTS_EXIST) {
      lightsInit();
    }

    SmartDashboard.putBoolean("use limelight", false);
    SmartDashboard.putBoolean("trust limelight", false);
    registerNamedCommands();
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

    new Trigger(AutoIntakeSup).whileTrue(new CollectFuel(drivetrain));
    new Trigger(crossBumpTowardsAllianceSup).whileTrue(new OverBump(drivetrain, 3));
    new Trigger(TrenchAllignSup).whileTrue(new LockYAxisForCrossing(drivetrain, ControllerForwardAxisSupplier, true, false));
    new Trigger(BumpAllignSup).whileTrue(new LockYAxisForCrossing(drivetrain, ControllerForwardAxisSupplier, false, true));
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
    //hood motor is now servo so cannot set speed
    //new Trigger(HoodUpSupplier).whileTrue(new RunCommand(()->shooter.setHoodMotor(0.2), shooter)).onFalse(new InstantCommand(()->shooter.setHoodMotor(0), shooter));
    //new Trigger(HoodDownSupplier).whileTrue(new RunCommand(()->shooter.setHoodMotor(-0.2), shooter)).onFalse(new InstantCommand(()->shooter.setHoodMotor(0), shooter));
    new Trigger(ForceHoodDownSupplier).whileTrue(new RunCommand(()-> shooter.setHoodAngleDown(), shooter));
    new Trigger(ShooterToggleSupplier).onTrue(new InstantCommand(()->manualShooterOn = !manualShooterOn));
    new Trigger(()->manualShooterOn).onTrue(new InstantCommand(()->shooter.set(0.2), shooter)).onFalse(new InstantCommand(()->shooter.stop(), shooter));
    new Trigger(AutoAimSupplier).whileTrue(new AimAtHub(drivetrain, shooter, ControllerSidewaysAxisSupplier, ControllerForwardAxisSupplier));
  }

  private void intakeInit() {
    intake = new Intake();
    
    new Trigger(IntakeWheelSup).whileTrue(new InstantCommand(()->intake.setWheelsVelocity(1), intake)).onFalse(new InstantCommand(()->intake.stopWheels(), intake));
    new Trigger(DeployIntakeSup).whileTrue(new InstantCommand(()->intake.deployIntake(), intake)).onFalse(new InstantCommand(()->intake.stopDeployer(), intake));
    new Trigger(RetractIntakeSup).whileTrue(new InstantCommand(()->intake.retractIntake(), intake)).onFalse(new InstantCommand(()->intake.stopDeployer(), intake));
  }

  private void climberInit() {
    climber = new Climber();

    new Trigger(ClimberDownSup).whileTrue(new InstantCommand(()->climber.climberDown(), climber)).onFalse(new InstantCommand(()->climber.stop(), climber));
    new Trigger(ClimberUpSup).whileTrue(new InstantCommand(()->climber.climberUp(), climber)).onFalse(new InstantCommand(()->climber.stop(), climber));
  }
  
  private void indexerInit() {
    indexer = new Indexer();
    new Trigger(IndexerSup).whileTrue(new FeedShooter(indexer, IndexerConstants.INDEXER_FEEDING_SPEED, hopper, HOPPER_ROLLER_SPEED));
    new Trigger(()->ShootIfAimedSup.getAsBoolean() && RobotState.getInstance().Aimed).whileTrue(new FeedShooter(indexer, Z_AXIS, hopper, HOPPER_ROLLER_SPEED));
  }

  private void lightsInit() {
    lights = new Lights();
    lights.setDefaultCommand(new LightsCommand(lights));
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
    if(DRIVE_TRAIN_EXISTS && SHOOTER_EXISTS){
      // new Trigger(AutoAimSupplier).whileTrue(new AimAtHub(drivetrain, shooter, ControllerSidewaysAxisSupplier, ControllerForwardAxisSupplier));
      new Trigger(ManualHubShotSup).whileTrue(new AimAtLocation(drivetrain, shooter, ControllerSidewaysAxisSupplier, ControllerForwardAxisSupplier, Location.Hub));
      new Trigger(ManualTowerShotSup).whileTrue(new AimAtLocation(drivetrain, shooter, ControllerSidewaysAxisSupplier, ControllerForwardAxisSupplier, Location.Tower));
      new Trigger(ManualLeftTrenchShotSup).whileTrue(new AimAtLocation(drivetrain, shooter, ControllerSidewaysAxisSupplier, ControllerForwardAxisSupplier, Location.LeftTrench));
      new Trigger(ManualRightTrenchShotSup).whileTrue(new AimAtLocation(drivetrain, shooter, ControllerSidewaysAxisSupplier, ControllerForwardAxisSupplier, Location.RightTrench));
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

  public void autonomousInit() {
    autoTimer = new Timer();
    autoTimer.reset();
    autoTimer.start();

    lights.blinkLights(LightsEnums.All, 255, 0, 0);
  }

  public void autonomousPeriodic() {
    if (autoTimer.hasElapsed(3.0)) {
      lights.setDefaultCommand(new LightsCommand(lights));
    }

  }

  public void runsWhenDisabled() {
    lights.breathingLights(LightsEnums.All, 255, 0, 255);
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

  private void registerNamedCommands(){
    Command AcrossBumpAwayFromAlliance = new SelectCommand<>(
      Map.ofEntries(
        Map.entry(true, new OverBump(drivetrain, 1.5)),
        Map.entry(false, new OverBump(drivetrain, -1.5))
      ),
      ()->DriverStation.getAlliance().get() == Alliance.Blue);
    Command AcrossBumpTowardsAlliance = new SelectCommand<>(
      Map.ofEntries(
        Map.entry(true, new OverBump(drivetrain, -1.5)), 
        Map.entry(false, new OverBump(drivetrain, 1.5))
      ),
      ()->DriverStation.getAlliance().get() == Alliance.Blue);
    NamedCommands.registerCommand("AcrossBumpAwayFromAlliance", AcrossBumpAwayFromAlliance);
    NamedCommands.registerCommand("AcrossBumpTowardsAlliance", AcrossBumpTowardsAlliance);
    NamedCommands.registerCommand("MoveToClimbingPose", new MoveToClimbingPose(drivetrain));
    NamedCommands.registerCommand("AimRobotMoving", new AimRobot(drivetrain, ControllerSidewaysAxisSupplier, ControllerForwardAxisSupplier, () -> RobotState.getInstance().aimingYaw));
    if(CLIMBER_EXISTS) {
      NamedCommands.registerCommand("ClimberArmUp", new ClimberArmUp(climber));
      NamedCommands.registerCommand("ClimberArmDown", new ClimberArmDown(climber));
      NamedCommands.registerCommand("AutomaticClimb", new AutomaticClimb(drivetrain, climber));
    } else {
      NamedCommands.registerCommand("ClimberArmUp", new InstantCommand(()->System.out.println("tried to run named command, but subsystem did not exist")));
      NamedCommands.registerCommand("ClimberArmDown", new InstantCommand(()->System.out.println("tried to run named command, but subsystem did not exist")));
      NamedCommands.registerCommand("AutomaticClimb", new InstantCommand(()->System.out.println("tried to run named command, but subsystem did not exist")));
    }
    if(INDEXER_EXISTS) {
      NamedCommands.registerCommand("RunIndexer", new FeedShooter(indexer, Z_AXIS, hopper, HOPPER_ROLLER_SPEED));
    } else {
      NamedCommands.registerCommand("RunIndexer", new InstantCommand(()->System.out.println("tried to run named command, but subsystem did not exist")));
    }
    if(SHOOTER_EXISTS) {
      NamedCommands.registerCommand("ShooterVelocity", new RunShooterVelocity(shooter, Z_AXIS));
    } else {
      NamedCommands.registerCommand("ShooterVelocity", new InstantCommand(()->System.out.println("tried to run named command, but subsystem did not exist")));
    }
    if(INTAKE_EXISTS) {
      NamedCommands.registerCommand("Intake", new RunIntake(intake));
      NamedCommands.registerCommand("Outtake", new Outtake(intake));
    } else {
      NamedCommands.registerCommand("Intake", new InstantCommand(()->System.out.println("tried to run named command, but subsystem did not exist")));
      NamedCommands.registerCommand("Outtake", new InstantCommand(()->System.out.println("tried to run named command, but subsystem did not exist")));
    }
  }
}
