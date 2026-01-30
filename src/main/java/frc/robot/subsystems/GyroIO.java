package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Minimal GyroIO interface exposing only the methods used by
 * {@code DrivetrainSubsystem} (Pigeon usage).
 */
public interface GyroIO {
    /** Initialize hardware resources (called once at startup). */
    void init();

    /**
     * Rotation2d representing the current robot yaw (used for odometry).
     * This matches the call site which previously used Pigeon2#getRotation2d().
     */
    Rotation2d getRotation2d();

    /** Get the current pitch (degrees). Matches Pigeon2#getPitch(). */
    double getPitch();

    /** Get the current roll (degrees). Matches Pigeon2#getRoll(). */
    double getRoll();

    /**
     * Get the angular velocity about the Z axis in world frame (degrees/sec).
     * Matches Pigeon2#getAngularVelocityZWorld().getValueAsDouble() usage.
     */
    double getAngularVelocityZWorld();
}