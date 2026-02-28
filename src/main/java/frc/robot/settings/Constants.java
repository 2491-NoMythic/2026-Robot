// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.settings;

import static frc.robot.settings.Constants.DriveConstants.FR_STEER_MOTOR_ID;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CommutationConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  private Constants() {
  }

  public static final class SubsystemsEnabled{
    public static final boolean SHOOTER_EXISTS = true;
    public static final boolean CLIMBER_EXISTS = true;
    public static final boolean INTAKE_EXISTS = true;
    public static final boolean INDEXER_EXISTS = true;
    public static final boolean DRIVE_TRAIN_EXISTS = true;
    public static final boolean LIMELIGHTS_EXIST = true;
    public static final boolean LIGHTS_EXIST = false;
    public static final boolean HOPPER_EXISTS = true;
  }

  public static final class ShooterConstants{
    public static final float SHOOTING_SPEED_MPS = 7;//not measured, just example
    public static final int SHOOTER_LEFT_MOTOR_ID = 9; 
    public static final int SHOOTER_RIGHT_MOTOR_ID = 10; 
    public static final int HOOD_LEFT_ACTUATOR_ID = 2;
    public static final int HOOD_RIGHT_ACTUATOR_ID = 3;
    public static final double SHOOTER_HEIGHT = 1; //IN METERS
    public static final double AUTO_AIM_ROBOT_kP = 0.125;
    public static final double AUTO_AIM_ROBOT_kI = 0;
    public static final double AUTO_AIM_ROBOT_kD = 0;
    public static final double HOOD_DOWN_POSITION = 0.2;
    public static final double HOOD_UP_POSITION = 0.8;
    public static TalonFXConfiguration SHOOTER_CONFIG = new TalonFXConfiguration()
      .withSlot0(new Slot0Configs()
        .withKV(0.17).withKP(1).withKI(0).withKD(0).withKS(0.675))
      .withCurrentLimits(new CurrentLimitsConfigs()
        .withSupplyCurrentLimitEnable(true).withSupplyCurrentLimit(50))
      .withMotorOutput(new MotorOutputConfigs()
        .withInverted(InvertedValue.Clockwise_Positive));
  }

  public static final class ClimberConstants{
    public static final int CLIMBER_MOTOR_ID = 16;
    public static final int HALL_EFFECT_ID = 6;
    public static final double CLIMBER_MAX_POSITION = 100;
    public static final double HALL_EFFECT_HEIGHT = 2491;
    public static TalonFXConfiguration CLIMBER_CONFIG = new TalonFXConfiguration()
      .withCurrentLimits(new CurrentLimitsConfigs()
        .withSupplyCurrentLimitEnable(true).withSupplyCurrentLimit(50))
      .withFeedback(new FeedbackConfigs()
        .withSensorToMechanismRatio((1.2 / 25) * Math.PI));//25 motor rotations = 1.2 pi inches of climber movement
  }

  public static final class IntakeConstants{
    public static final double INTAKE_SPEED = 0.8;
    public static final int INTAKE_WHEELS_ID = 14;
    public static final int INTAKE_DEPLOYER_ID = 13;
    public static TalonFXConfiguration INTAKE_DEPLOYER_CONFIG = new TalonFXConfiguration()
      .withSlot0(new Slot0Configs()
        .withKV(0).withKP(0).withKI(0).withKD(0))
      .withCurrentLimits(new CurrentLimitsConfigs()
        .withSupplyCurrentLimitEnable(true).withSupplyCurrentLimit(50))
      .withHardwareLimitSwitch(new HardwareLimitSwitchConfigs()
        .withForwardLimitEnable(true)
        .withForwardLimitRemoteSensorID(FR_STEER_MOTOR_ID)
        .withForwardLimitSource(ForwardLimitSourceValue.RemoteTalonFX)
        .withForwardLimitType(ForwardLimitTypeValue.NormallyOpen)
        .withReverseLimitEnable(true)
        .withReverseLimitRemoteSensorID(FR_STEER_MOTOR_ID)
        .withReverseLimitSource(ReverseLimitSourceValue.RemoteTalonFX)
        .withReverseLimitType(ReverseLimitTypeValue.NormallyOpen));
    public static TalonFXSConfiguration INTAKE_WHEELS_CONFIG = new TalonFXSConfiguration()
      .withCommutation(new CommutationConfigs()
        .withMotorArrangement(MotorArrangementValue.Minion_JST))
      .withSlot0(new Slot0Configs()
        .withKV(0.095).withKS(0.37).withKP(0.1).withKI(0).withKD(0));
  }

  public static final class IndexerConstants{
    public static final int INDEXER_MOTOR_1_ID= 11;//right motor (leader)
    public static final int INDEXER_MOTOR_2_ID= 12;//left motor (follower)
    public static final double INDEXER_FEEDING_SPEED = 0.5;
    public static final double INDEXER_FEEDING_RPS = 10;

    public static TalonFXConfiguration INDEXER_RIGHT_CONFIG = new TalonFXConfiguration()
      .withSlot0(new Slot0Configs()
        .withKV(0.091).withKP(0.1).withKI(0).withKD(0).withKS(0.5))
      .withMotorOutput(new MotorOutputConfigs()
        .withInverted(InvertedValue.Clockwise_Positive));
    
      public static TalonFXConfiguration INDEXER_LEFT_CONFIG = new TalonFXConfiguration()
        .withMotorOutput(new MotorOutputConfigs()
          .withInverted(InvertedValue.CounterClockwise_Positive));

  }

  public static final class HopperConstants{
    public static final int HOPPER_MOTOR_ID = 15;
    public static final double HOPPER_ROLLER_SPEED = 0.3;

    public static final int TOP_LEFT_SENSOR_ID = 2491;
    public static final int LIMIT_SWITCH_1_ID = 2491;
    public static final int LIMIT_SWITCH_2_ID = 2491;

    public static final double DISTANCE_SENSOR_RANGE = 2491;

    public static TalonFXConfiguration HOPPER_CONFIG = new TalonFXConfiguration()
      .withSlot0(new Slot0Configs()
        .withKV(0.115).withKP(0.1).withKI(0).withKD(0).withKS(0.65))
      .withMotorOutput(new MotorOutputConfigs()
        .withInverted(InvertedValue.Clockwise_Positive))
      .withVoltage(new VoltageConfigs()
        .withPeakForwardVoltage(5)
        .withPeakReverseVoltage(-5));
  }

  public static final class Field{
    public static final Translation3d BLUE_HUB_COORDINATE = new Translation3d(4.6, 4, 1.8);
    public static final Translation3d RED_HUB_COORDINATE = new Translation3d(11.9, 4, 1.8);
  }

  public static final class AimAtLocationConstants {
    public static final int HUB_ROBOT_ANGLE = 0;
    public static final int HUB_HOOD_ANGLE = 5;
    public static final int L_TRENCH_ROBOT_ANGLE = -80;
    public static final int R_TRENCH_ROBOT_ANGLE = 80;
    public static final int TRENCH_HOOD_ANGLE = 2491;
    public static final int TOWER_ROBOT_ANGLE = 0;
    public static final int TOWER_HOOD_ANGLE = 30;
  }

  public static final class DriveConstants {
    public static final Pose2d DRIVE_ODOMETRY_ORIGIN = new Pose2d(5.0, 5.0, new Rotation2d());

    public static final PIDController DRIVE_TO_POSE_X_CONTROLLER = getTranslationPIDController();
    public static final PIDController DRIVE_TO_POSE_Y_CONTROLLER = getTranslationPIDController();

    /** The bumper-to-bumper width of the robot. */
    public static PIDController getTranslationPIDController() {
      PIDController transltionController = new PIDController(5, 0, 0);
      transltionController.setIZone(0.025);
      transltionController.setIntegratorRange(-0.25, 0.25);
      return transltionController;
    }
    public static final double DRIVETRAIN_ROBOT_WIDTH_METERS = 0.60325;
    //old drivetrain:
    //public static final double DRIVETRAIN_ROBOT_WIDTH_METERS = 0.83;
    /**
     * The left-to-right distance between the drivetrain wheels Should be measured
     * from center to
     * center.
     */
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.50165;
    //old drivetrain:
    //public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.58;
    /**
     * The front-to-back distance between the drivetrain wheels. Should be measured
     * from center to
     * center.
     */
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.63;

    /** The diameter of the module's wheel in meters. */
    public static final double DRIVETRAIN_WHEEL_DIAMETER = 0.1016; // 0.098;

    /**
     * The overall drive reduction of the module. Multiplying motor rotations by
     * this value should
     * result in wheel rotations. these numbers are just gear ratios that are used.
     * Ask build team
     * about these.
     */
    public static final double DRIVETRAIN_DRIVE_REDUCTION = (13.0 / 50.0) * (28.0 / 16.0) * (15.0 / 45.0);

    /**
     * Whether the drive motor should be counterclockwise or clockwise positive. If
     * there is an odd
     * number of gear reductions this is typically clockwise-positive.
     */
    public static final InvertedValue DRIVETRAIN_DRIVE_INVERTED = InvertedValue.Clockwise_Positive;

    /**
     * The overall steer reduction of the module. Multiplying motor rotations by
     * this value should
     * result in wheel rotations.
     */
    public static final double DRIVETRAIN_STEER_REDUCTION = (16.0 / 43.0) * (10.0 / 60.0);
    // old drive train use: 
    //public static final double DRIVETRAIN_STEER_REDUCTION = (14.0 / 50.0) * (10.0 / 60.0);
    /**
     * Whether the steer motor should be counterclockwise or clockwise positive. If
     * there is an odd
     * number of gear reductions this is typically clockwise-positive.
     */
    public static final InvertedValue DRIVETRAIN_STEER_INVERTED = InvertedValue.CounterClockwise_Positive;
    //old drivetrain:
    //    public static final InvertedValue DRIVETRAIN_STEER_INVERTED = InvertedValue.Clockwise_Positive;

    /**
     * How many meters the wheels travel per rotation.
     *
     * <p>
     * Multiply rotations by this to get meters.
     *
     * <p>
     * Divide meters by this to get rotations.
     */
    public static final double DRIVETRAIN_ROTATIONS_TO_METERS = (DRIVETRAIN_WHEEL_DIAMETER * Math.PI);

    /**
     * The maximum velocity of the robot in meters per second.
     *
     * <p>
     * This is a measure of how fast the robot should be able to drive in a straight
     * line.
     */
    /*
     * FIXME Measure the drivetrain's maximum velocity or calculate the theoretical.
     * The formula for calculating the theoretical maximum velocity is:
     * <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> *
     * pi
     */
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 8000.0 / 60.0 * DRIVETRAIN_DRIVE_REDUCTION
        * DRIVETRAIN_WHEEL_DIAMETER * Math.PI;
    /**
     * The drive motor sensor value at a 100% duty cycle output in a straight line.
     */
    public static final double MAX_VELOCITY_RPS_EMPIRICAL = 15.697;
    /**
     * The maximum angular velocity of the robot in radians per second.
     *
     * <p>
     * This is a measure of how fast the robot can rotate in place.
     */
    // Here we calculate the theoretical maximum angular velocity. You can also
    // replace this with a
    // measured amount.
    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND
        / Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);

    public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        // Front left
        new Translation2d(
            DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
        // Front right
        new Translation2d(
            DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
        // Back left
        new Translation2d(
            -DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
        // Back right
        new Translation2d(
            -DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0));

    public static final String DRIVETRAIN_SMARTDASHBOARD_TAB = "Drivetrain";
    public static final String CANIVORE_DRIVETRAIN = "Swerve";
    public static final int DRIVETRAIN_PIGEON_ID = 0;

    public static final int FL_DRIVE_MOTOR_ID = 1;
    public static final int FL_STEER_MOTOR_ID = 2;
    public static final int FL_STEER_ENCODER_ID = 1;
    public static final Rotation2d FL_STEER_OFFSET = Rotation2d.fromRotations(0.272217);

    public static final int FR_DRIVE_MOTOR_ID = 3;
    public static final int FR_STEER_MOTOR_ID = 4;
    public static final int FR_STEER_ENCODER_ID = 2;
    public static final Rotation2d FR_STEER_OFFSET = Rotation2d.fromRotations(0.41333);

    public static final int BL_DRIVE_MOTOR_ID = 5;
    public static final int BL_STEER_MOTOR_ID = 6;
    public static final int BL_STEER_ENCODER_ID = 3;
    public static final Rotation2d BL_STEER_OFFSET = Rotation2d.fromRotations(-0.11792);

    public static final int BR_DRIVE_MOTOR_ID = 7;
    public static final int BR_STEER_MOTOR_ID = 8;
    public static final int BR_STEER_ENCODER_ID = 4;
    public static final Rotation2d BR_STEER_OFFSET = Rotation2d.fromRotations(0.403809);

    // Drive Motor
    public static final double k_DRIVE_P = 0.03;
    public static final double k_DRIVE_I = 0;
    public static final double k_DRIVE_D = 0;
    public static final double k_DRIVE_FF_S = 0;
    public static final double k_DRIVE_FF_V = 0;
    public static final double DRIVE_DEADBAND_MPS = 0.01;
    public static final double DRIVE_MOTOR_RAMP = 0.1;
    public static final double DRIVE_CURRENT_LIMIT = 30;

    // Steer Motor
    /**
     * The maximum velocity of the steer motor.
     *
     * <p>
     * This is the limit of how fast the wheels can rotate in place.
     */
    public static final double MAX_STEER_VELOCITY_RADIANS_PER_SECOND = Math.PI; // 1/2 rotation per second.
    /**
     * The maximum acceleration of the steer motor.
     *
     * <p>
     * This is the limit of how fast the wheels can change rotation speed.
     */
    public static final double MAX_STEER_ACCELERATION_RADIANS_PER_SECOND_SQUARED = 2 * Math.PI;

    public static final double k_STEER_P = 8;
    public static final double k_STEER_I = 0;
    public static final double k_STEER_D = 0;
    public static final double k_STEER_FF_S = 0.0;
    public static final double k_STEER_FF_V = 0.0;

    // Auto PID loops
    // twin pid controllers that control the x and y robot movements.
    public static final double k_XY_P = 7; // *2.5;
    public static final double k_XY_I = 0.3;
    public static final double k_XY_D = 0.0;

    public static final double k_THETA_P = 4;
    public static final double k_THETA_I = 5.0;
    public static final double k_THETA_D = 0.0;
    public static final double k_THETA_TOLORANCE_DEGREES = 2.0;
    public static final double k_THETA_TOLORANCE_DEG_PER_SEC = 10;

    public static final double AUTO_AIM_ROBOT_kP = 0.125;
    public static final double AUTO_AIM_ROBOT_kI = 0.00;
    public static final double AUTO_AIM_ROBOT_kD = 0.00;

    public static final double ROBOT_ANGLE_TOLERANCE = 0.5;

    public static final PathConstraints DEFAULT_PATH_CONSTRAINTS = new PathConstraints(1, 1, Math.toRadians(360),
        Math.toRadians(360));
  }

  public static final class CTREConfigs {
    // Drive motor.
    private static TalonFXConfiguration getDriveMotorConfig() {
      TalonFXConfiguration driveMotorConfig = new TalonFXConfiguration();
      driveMotorConfig.Feedback.SensorToMechanismRatio = 1 / DriveConstants.DRIVETRAIN_DRIVE_REDUCTION;
      driveMotorConfig.MotorOutput.Inverted = DriveConstants.DRIVETRAIN_DRIVE_INVERTED;
      driveMotorConfig.MotorOutput.DutyCycleNeutralDeadband = 0.0;
      driveMotorConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = DriveConstants.DRIVE_MOTOR_RAMP;
      driveMotorConfig.Slot0.kP = DriveConstants.k_DRIVE_P * 12;
      driveMotorConfig.Slot0.kI = DriveConstants.k_DRIVE_I * 12;
      driveMotorConfig.Slot0.kD = DriveConstants.k_DRIVE_D * 12;
      driveMotorConfig.Slot0.kS = DriveConstants.k_DRIVE_FF_S;
      driveMotorConfig.Slot0.kV = DriveConstants.k_DRIVE_FF_V;
      // TODO: ADD DUPLICATE CONSTANTS
      driveMotorConfig.Voltage.PeakForwardVoltage = 12;
      driveMotorConfig.Voltage.PeakReverseVoltage = -12;
      driveMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      driveMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
      driveMotorConfig.CurrentLimits.StatorCurrentLimitEnable = false;
      /*
       * the following does this:
       * the current is always limited to 50 Amps. If the limit has been needed (motor
       * is demanding more than 50, but limit stops it) for more than 0.8 seconds,
       * than
       * limit changes to DRIVE_CURRENT_LIMIT, which is 30 amps. this lower limit is
       * enabled until the demanded current drops below the lower limit, then the 50
       * amp limit is enabled
       */
      driveMotorConfig.CurrentLimits.SupplyCurrentLimit = 50;
      driveMotorConfig.CurrentLimits.SupplyCurrentLowerLimit = DriveConstants.DRIVE_CURRENT_LIMIT;
      driveMotorConfig.CurrentLimits.SupplyCurrentLowerTime = 0.8;
      return driveMotorConfig;
    }

    // Steer motor.
    private static TalonFXConfiguration getSteerMotorConfig() {
      TalonFXConfiguration steerMotorConfig = new TalonFXConfiguration();
      steerMotorConfig.Feedback.RotorToSensorRatio = 1 / DriveConstants.DRIVETRAIN_STEER_REDUCTION;
      steerMotorConfig.MotorOutput.Inverted = DriveConstants.DRIVETRAIN_STEER_INVERTED;
      // steerMotorConfig.MotorOutput.DutyCycleNeutralDeadband = 0.05;
      steerMotorConfig.Slot0.kP = DriveConstants.k_STEER_P;
      steerMotorConfig.Slot0.kI = DriveConstants.k_STEER_I;
      steerMotorConfig.Slot0.kD = DriveConstants.k_STEER_D;
      steerMotorConfig.Slot0.kS = DriveConstants.k_STEER_FF_S;
      steerMotorConfig.Slot0.kV = DriveConstants.k_STEER_FF_V;
      steerMotorConfig.Voltage.PeakForwardVoltage = 12;
      steerMotorConfig.Voltage.PeakReverseVoltage = -12;
      steerMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;

      steerMotorConfig.ClosedLoopGeneral.ContinuousWrap = true;
      return steerMotorConfig;
    }

    // Steer encoder.
    private static CANcoderConfiguration getSteerEncoderConfig() {
      CANcoderConfiguration steerEncoderConfig = new CANcoderConfiguration();
      steerEncoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
      return steerEncoderConfig;
    }

    // Pigeon 2.
    private static Pigeon2Configuration getPigeon2Config() {
      Pigeon2Configuration pigeon2Config = new Pigeon2Configuration();
      pigeon2Config.MountPose.MountPosePitch = 0;
      pigeon2Config.MountPose.MountPoseRoll = 0;
      pigeon2Config.MountPose.MountPoseYaw = 0;
      return pigeon2Config;
    }

    public static final TalonFXConfiguration driveMotorConfig = getDriveMotorConfig();
    public static final TalonFXConfiguration steerMotorConfig = getSteerMotorConfig();
    public static final CANcoderConfiguration steerEncoderConfig = getSteerEncoderConfig();
    public static final Pigeon2Configuration pigeon2Config = getPigeon2Config();

  }

  public final class XboxDriver {
    private XboxDriver() {
    }

    public static final int DRIVE_CONTROLLER_ID = 0;
    public static final int OPERATOR_CONTROLLER_ID = 1;
    /**
     * Left stick Y-axis.
     *
     * <p>
     * Left = -1 || Right = 1
     */
    public static final int X_AXIS = 0;
    /**
     * Left stick X-axis.
     *
     * <p>
     * Forwards = -1 || Backwards = 1
     */
    public static final int Y_AXIS = 1;
    /**
     * Right stick Y-axis for xbox controllers.
     *
     * <p>
     * Left = -1 || Right = 1
     */
    public static final int XBOX_Z_AXIS = 4;
    /**
     * Right stick Z-axis.
     *
     * <p>
     * Left = -1 || Right = 1
     */
    public static final int Z_AXIS = 4;
    /**
     * Right stick Z-rotate.
     *
     * <p>
     * Forwards = -1 || Backwards = 1
     */
    public static final int Z_ROTATE = 5;
    /** Value used to differentiate between angle 0 and rest position. */
    public static final double NO_INPUT = 404;

    public static final double DEADBAND_NORMAL = 0.03;
    public static final double DEADBAND_LARGE = 0.1;
  }

  public final class Vision {
    public static final String APRILTAG_LIMELIGHTA_NAME = "limelight-aprila";
    public static final String APRILTAG_LIMELIGHTB_NAME = "limelight-aprilb";
    public static final String APRILTAG_LIMELIGHTC_NAME = "limelight-aprilc";
    public static final String OBJ_DETECTION_LIMELIGHT_NAME = "limelight-neural";

    public static final String LIMELIGHT_SHUFFLEBOARD_TAB = "Vision";

    public static final double ALLOWABLE_POSE_DIFFERENCE = 0.5;
    public static final double MAX_TAG_DISTANCE = 3;

    public static final Translation2d FIELD_CORNER = new Translation2d(17.54, 8.02);

    // how many degrees back is your limelight rotated from perfectly vertical?
    public static final double limelightMountAngleDegrees = 2491;
    // distance from the center of the Limelight lens to the floor
    public static final double limelightLensHeightInches = 2491;
    // height of april tags from the floor in meters
    public static final double AprilTagHeight = 1.335;
  }

  public final class LightConstants {
    public static final int CANDLE_ID = 2491;
      public static final int ALL_LIGHT_START = 0;
      public static final int ALL_LIGHT_END = 60;
  }
}
