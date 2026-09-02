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
import static frc.robot.settings.Constants.HopperConstants.HOPPER_ROLLER_SPEED_RPS;
import static frc.robot.settings.Constants.ShooterConstants.HOOD_UP_POSITION;
import static frc.robot.settings.Constants.SubsystemsEnabled.DRIVE_TRAIN_EXISTS;
import static frc.robot.settings.Constants.SubsystemsEnabled.HOPPER_EXISTS;
import static frc.robot.settings.Constants.SubsystemsEnabled.INDEXER_EXISTS;
import static frc.robot.settings.Constants.SubsystemsEnabled.INTAKE_EXISTS;
import static frc.robot.settings.Constants.SubsystemsEnabled.LIGHTS_EXIST;
import static frc.robot.settings.Constants.SubsystemsEnabled.SAFE_MODE_IS_ON;
import static frc.robot.settings.Constants.SubsystemsEnabled.SHOOTER_EXISTS;
import static frc.robot.settings.Constants.XboxDriver.DRIVE_CONTROLLER_ID;
import static frc.robot.settings.Constants.XboxDriver.OPERATOR_CONTROLLER_ID;
import static frc.robot.settings.Constants.XboxDriver.X_AXIS;
import static frc.robot.settings.Constants.XboxDriver.Y_AXIS;
import static frc.robot.settings.Constants.XboxDriver.Z_AXIS;

import java.io.IOException;
import java.util.HashMap;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.AimHood;
import frc.robot.Commands.AimRobot;
import frc.robot.Commands.Drive;
import frc.robot.Commands.DriveConstantSpeed;
import frc.robot.Commands.Expand;
import frc.robot.Commands.Outtake;
import frc.robot.Commands.FeedShooter;
import frc.robot.Commands.PulseIntake;
import frc.robot.Commands.FeedShooterAntiHopperStall;
import frc.robot.Commands.MoveIntakeUp;
import frc.robot.Commands.RunIntake;
import frc.robot.settings.Constants.ShooterConstants;
import frc.robot.settings.OdometryUpdatingState;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lights;
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
  private Intake intake;
  private Indexer indexer;
  private Hopper hopper;
  @SuppressWarnings("unused")
  private Lights lights;
  private Drive defaultDriveCommand;
  private SendableChooser<Command> autoChooser;
  private SendableChooser<Double> safeModeChooser; // creates a changable option on elastic for safemode
  private final XboxController driveController;
  private final XboxController operatorController;
  private Timer autoTimer;

  DoubleSupplier ControllerForwardAxisSupplier;
  DoubleSupplier ControllerSidewaysAxisSupplier;
  DoubleSupplier ControllerZAxisSupplier;
  DoubleSupplier SafeControllerForwardAxisSupplier;
  DoubleSupplier SafeControllerSidewaysAxisSupplier;
  DoubleSupplier SafeControllerZAxisSupplier;
  BooleanSupplier ZeroGyroSup;
  BooleanSupplier AimRobotMovingSup;
  BooleanSupplier TrenchAllignSup;
  BooleanSupplier BumpAllignSup;
  BooleanSupplier RetractIntakeSup;
  BooleanSupplier IntakeBackwardsSup;
  BooleanSupplier DeployIntakeSup;
  BooleanSupplier AutoIntakeSup;
  BooleanSupplier IntakeWheelSup;
  BooleanSupplier PulseIntakeSup;
  BooleanSupplier ShooterOnSup;
  BooleanSupplier ShooterOffSup;
  BooleanSupplier HoodUpSupplier;
  BooleanSupplier HoodDownSupplier;
  BooleanSupplier HopperWheelsForwardSup;
  BooleanSupplier HopperWheelsBackwardSup;
  BooleanSupplier IndexerSup;
  BooleanSupplier AutoAimSupplier;
  BooleanSupplier ShootIfAimedSup;
  BooleanSupplier ForceHoodDownSupplier;
  BooleanSupplier crossBumpTowardsAllianceSup;
  BooleanSupplier ManualHubShotSup;
  BooleanSupplier ManualTowerShotSup;
  BooleanSupplier ManualLeftTrenchShotSup;
  BooleanSupplier ManualRightTrenchShotSup;
  BooleanSupplier ResetQuestIntakeInSup;
  BooleanSupplier ResetQuestIntakeOutSup;
  BooleanSupplier ManualRightCornerShotSup;
  BooleanSupplier ManualLeftCornerShotSup;
  BooleanSupplier DrivetrainXPositionSup;
  BooleanSupplier AgitateFuelSup;



  public static HashMap<String, Command> eventMap;

  public RobotContainer() {

    RobotState.getInstance().odometryUpdatingState = OdometryUpdatingState.drivetrainAndLimelights;
    
    autoTimer = new Timer();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    driveController = new XboxController(DRIVE_CONTROLLER_ID);
    operatorController = new XboxController(OPERATOR_CONTROLLER_ID);
    autoChooser = new SendableChooser<>();
    eventMap = new HashMap<>();

    // Drive controls
    if (SAFE_MODE_IS_ON) {   // this code will only run if the constant SAFE_MODE_IS_ON is set to true, 
      safeModeChooser = new SendableChooser<>();                  // creates a new instance of SafeModeChooser which can be sent to elastic
      safeModeChooser.addOption("Cheetah", 0.35);      // creates Cheetah mode in which speed is set to 0.5 of normal speed
      safeModeChooser.addOption("Dog", 0.2);
      safeModeChooser.addOption("Turtle", 0.1);
      safeModeChooser.addOption("Full", 1.0);
      SmartDashboard.putData("Safe Mode", safeModeChooser);
    }
    ControllerSidewaysAxisSupplier = () -> modifyAxis(-driveController.getRawAxis(X_AXIS), 0);
    ControllerForwardAxisSupplier = () -> modifyAxis(-driveController.getRawAxis(Y_AXIS), 0);
    ControllerZAxisSupplier = () -> modifyAxis(-driveController.getRawAxis(Z_AXIS), 0);
    SafeControllerSidewaysAxisSupplier = () -> getSpeedMultiplier() * modifyAxis(-driveController.getRawAxis(X_AXIS), 0);
      //Makes a seperate supplier that slows driving speed
    SafeControllerForwardAxisSupplier = () -> getSpeedMultiplier() * modifyAxis(-driveController.getRawAxis(Y_AXIS), 0);
      //Makes a seperate supplier that slows driving speed 
    SafeControllerZAxisSupplier = () -> getSpeedMultiplier() * modifyAxis(-driveController.getRawAxis(Z_AXIS), 0);
    ZeroGyroSup = driveController::getStartButton;
    AutoAimSupplier = () -> driveController.getLeftTriggerAxis() >= 0.5;
    AutoIntakeSup = driveController::getXButton;
    DrivetrainXPositionSup = () -> driveController.getAButton();


    //Shooter controls
    IndexerSup = ()-> driveController.getRightTriggerAxis() > 0.5;
    ForceHoodDownSupplier = operatorController::getBackButton;

    HoodUpSupplier = () -> operatorController.getPOV() == 0;
    HoodDownSupplier = () -> operatorController.getRightTriggerAxis() > 0.5;
    ShooterOnSup = ()-> operatorController.getPOV() == 90;
    ShooterOffSup = ()-> operatorController.getPOV() == 270;
    ManualHubShotSup = operatorController::getYButton;
    ManualTowerShotSup = operatorController::getAButton;
    ManualLeftTrenchShotSup = operatorController::getXButton;
    ManualRightTrenchShotSup = operatorController::getBButton;
    ManualLeftCornerShotSup = operatorController::getLeftBumperButton;
    ManualRightCornerShotSup = operatorController::getRightBumperButton;

    //Shooting Command is Right Trigger on drive controller. 

    //intake controls
    RetractIntakeSup = operatorController::getLeftStickButton;
    DeployIntakeSup = operatorController::getRightStickButton;
    IntakeWheelSup = driveController::getLeftBumperButton;
    IntakeBackwardsSup = driveController::getRightBumperButton;
    PulseIntakeSup = ()->operatorController.getPOV() > 134 && operatorController.getPOV() < 226;
    IntakeBackwardsSup = driveController::getRightBumperButton;
    AgitateFuelSup = ()->operatorController.getLeftTriggerAxis() > 0.5;

    //hopper controls
    HopperWheelsForwardSup = ()-> false;//operatorController.getPOV() == 270;
    HopperWheelsBackwardSup = ()-> false;//operatorController.getPOV() == 180;

    //Trench Controls
    TrenchAllignSup = driveController::getLeftStickButton; //NOT FINAL THIS IS BATCRAP INSANE
    BumpAllignSup = driveController::getRightStickButton; //NOT FINAL THIS IS BATCRAP INSANE

    crossBumpTowardsAllianceSup = driveController::getYButton;
    ShootIfAimedSup = ()->false;

    //QuestNav Controls
    ResetQuestIntakeInSup = driveController::getBackButton;

    if (DRIVE_TRAIN_EXISTS) {
      driveTrainInit();
      configureDriveTrain(drivetrain::resetOdometry);
    }

    if(HOPPER_EXISTS) {
      hopperInit();
    }

    if (SHOOTER_EXISTS) {
      shooterInit();
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
    configureBindings();
  }

  private double getSpeedMultiplier() {  // if safe mode is off, this will always return 1.0 and the robot will drive normally, if safemode is on it will return the number of the mode that is selected
    if (SAFE_MODE_IS_ON) {
      return safeModeChooser.getSelected();
    } else {
      return 1.0;
    }
  }

  private void driveTrainInit() {
    drivetrain = new DrivetrainSubsystem();

    defaultDriveCommand = new Drive(
        drivetrain,
        () -> false,
        SafeControllerForwardAxisSupplier,
        SafeControllerSidewaysAxisSupplier,
        SafeControllerZAxisSupplier);
    drivetrain.setDefaultCommand(defaultDriveCommand);

    new Trigger(DrivetrainXPositionSup).whileTrue(drivetrain.run(()->drivetrain.pointWheelsInward()));

    SmartDashboard.putData("DriveConstant1", new DriveConstantSpeed(drivetrain, 1, 2));
    SmartDashboard.putData("DriveConstant2", new DriveConstantSpeed(drivetrain, 2, 2));
    SmartDashboard.putData("DriveConstant3", new DriveConstantSpeed(drivetrain, 3, 1.5));
  }
 
  private void configureDriveTrain(Consumer<Pose2d> resetOdometryConsumer) {
    try {
      AutoBuilder.configure(
          drivetrain::getPose, // Pose2d supplier
          resetOdometryConsumer, // Pose2d consumer, used to reset odometry at the beginning of auto
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

  private void shooterInit() {
    shooter = new Shooter();
    shooter.setDefaultCommand(new AimHood(shooter));
    new Trigger(HoodUpSupplier).whileTrue(new RunCommand(()->shooter.setDesiredHoodAngle(HOOD_UP_POSITION), shooter));
    new Trigger(HoodDownSupplier).whileTrue(new RunCommand(()-> shooter.setDesiredHoodAngle(ShooterConstants.HOOD_DOWN_POSITION), shooter));
    new Trigger(ShooterOnSup).onTrue(new InstantCommand(()->shooter.shooterOn(), shooter));
    new Trigger(ShooterOffSup).onTrue(new InstantCommand(()->shooter.stop(), shooter));

    SmartDashboard.putData("TESTING/HoodTo28Degrees", new RunCommand(()->shooter.setDesiredHoodAngle(25), shooter));
  }

  private void hopperInit() {
    hopper = new Hopper();

    new Trigger(HopperWheelsForwardSup).onTrue(new InstantCommand(()->hopper.setVelocity(HOPPER_ROLLER_SPEED_RPS), hopper )).onFalse(new InstantCommand(()->hopper.setHopperRoller(0), hopper));
    new Trigger(HopperWheelsBackwardSup).onTrue(new InstantCommand(()->hopper.setVelocity(-HOPPER_ROLLER_SPEED_RPS), hopper )).onFalse(new InstantCommand(()->hopper.setHopperRoller(0), hopper));
  }

  private void intakeInit() {
    intake = new Intake();
    
    new Trigger(DeployIntakeSup).whileTrue(new Expand(intake));
    new Trigger(RetractIntakeSup).whileTrue(new InstantCommand(()->intake.retractIntake(), intake));
    new Trigger(IntakeBackwardsSup).whileTrue(intake.run(()->intake.setVelocity(-45))).onFalse(new InstantCommand(()->intake.stopWheels(), intake));
    new Trigger(PulseIntakeSup).whileTrue(new PulseIntake(intake));
    new Trigger(IntakeBackwardsSup).whileTrue(intake.run(()->intake.setVelocity(-45))).onFalse(new InstantCommand(()->intake.stopWheels(), intake));
    new Trigger(AgitateFuelSup).whileTrue(new MoveIntakeUp(intake, 0.5));
    
    if(HOPPER_EXISTS) {
      new Trigger(()->IntakeWheelSup.getAsBoolean() && !RobotState.getInstance().feedingShooter).whileTrue(new RunIntake(intake, hopper));
    } else {
      new Trigger(()->IntakeWheelSup.getAsBoolean() && !RobotState.getInstance().feedingShooter).whileTrue(new RunCommand(()->intake.feedHopper(), intake)).onFalse(new InstantCommand(()->intake.stopWheels(), intake));
    }
    new Trigger(()->IntakeWheelSup.getAsBoolean() && RobotState.getInstance().feedingShooter).whileTrue(new RunCommand(()->intake.feedHopper(), intake)).onFalse(new InstantCommand(()->intake.stopWheels(), intake));
  }
  
  private void indexerInit() {
    indexer = new Indexer();
  }

  private void lightsInit() {
    lights = new Lights();
  }

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

    if(INTAKE_EXISTS && INDEXER_EXISTS && HOPPER_EXISTS) {
      new Trigger(()->ShootIfAimedSup.getAsBoolean() && RobotState.getInstance().Aimed).whileTrue(new FeedShooter(indexer, hopper, intake));  
      new Trigger(IndexerSup).whileTrue(new FeedShooter(indexer, hopper, intake));
      new Trigger(IndexerSup).whileTrue(new InstantCommand(()->System.out.println("IndexerSupPressed")));
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
    //THIS METHOD IS NEVER CALLED
    
    autoTimer.reset();
    autoTimer.start();

    //lights.blinkLights(LightsEnums.All, 255, 0, 0);
  }

  public void autonomousPeriodic() {
    RobotState.getInstance().lightsRobotDisabled = false;
  }

  public void disabledPeriodic() {
    RobotState.getInstance().lightsRobotDisabled = true;
    //lights.breathingLights(LightsEnums.All, 255, 0, 255);
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

  public void teleopPeriodic() {
    SmartDashboard.putNumber("RobotAngle", drivetrain.getOdometryRotation().getDegrees());
    SmartDashboard.putNumber("GetPose", drivetrain.getPose().getRotation().getDegrees());

    RobotState.getInstance().lightsRobotDisabled = false;
  }

  @SuppressWarnings("unused")
  private void registerNamedCommands(){
    if(INDEXER_EXISTS && HOPPER_EXISTS) {
      NamedCommands.registerCommand("RunIndexer", new ParallelCommandGroup(
        new AimRobot(drivetrain, ControllerZAxisSupplier, ControllerSidewaysAxisSupplier, ()->RobotState.getInstance().aimingYaw),
        new FeedShooter(indexer, hopper, intake)));
      NamedCommands.registerCommand("FeedShooterAntiStall", new FeedShooterAntiHopperStall(hopper, indexer));
    } else {
      NamedCommands.registerCommand("RunIndexer", new InstantCommand(()->System.out.println("tried to run named command, but subsystem did not exist")));
      NamedCommands.registerCommand("FeedShooterAntiStall", new InstantCommand(()->System.out.println("tried to run named command, but subsystem did not exist")));
    }
    if(SHOOTER_EXISTS) {
      NamedCommands.registerCommand("ShooterOn", new InstantCommand(()->shooter.shooterOn(), shooter));
      NamedCommands.registerCommand("WaitUntilShooterIsSpooled", new WaitUntilCommand(()->shooter.isAtSpeed()));
    } else {
      NamedCommands.registerCommand("ShooterOn", new InstantCommand(()->System.out.println("tried to run named command, but subsystem did not exist")));
      NamedCommands.registerCommand("WaitUntilShooterIsSpooled", new InstantCommand(()->System.out.println("tried to run named command, but subsystem did not exist")));
    }
    if(INTAKE_EXISTS) {
      NamedCommands.registerCommand("Outtake", new Outtake(intake));
      NamedCommands.registerCommand("Expand", new Expand(intake).withDeadline(new WaitCommand(1)));
      NamedCommands.registerCommand("RunOnlyIntake", new InstantCommand(()->intake.feedHopper(), intake));
      NamedCommands.registerCommand("AgitateFuel", new MoveIntakeUp(intake, 1));

      if(HOPPER_EXISTS) {
        NamedCommands.registerCommand("Intake", new RunIntake(intake, hopper));
      }
    } else {
      NamedCommands.registerCommand("Intake", new InstantCommand(()->System.out.println("tried to run named command, but subsystem did not exist")));
      NamedCommands.registerCommand("AgitateFuel", new InstantCommand(()->System.out.println("tried to run named command, but subsystem did not exist")));
      NamedCommands.registerCommand("RunOnlyIntake", new InstantCommand(()->System.out.println("tried to run named command, but subsystem did not exist")));
      NamedCommands.registerCommand("Outtake", new InstantCommand(()->System.out.println("tried to run named command, but subsystem did not exist")));
      NamedCommands.registerCommand("Expand", new InstantCommand(()->System.out.println("tried to run named command, but subsystem did not exist")));
    }
  }
}
