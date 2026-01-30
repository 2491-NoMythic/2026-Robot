package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * GyroIO implementation that wraps a CTRE Pigeon2 device.
 */
public class GyroIOPigeon2 implements GyroIO {
    private final Pigeon2 pigeon;

    /**
     * Create a new Pigeon2 wrapper.
     *
     * @param deviceId CAN device ID for the Pigeon2
     * @param canBus   CAN bus name (e.g. CANivore name or "" for default)
     */
    public GyroIOPigeon2(int deviceId, String canBus) {
        this.pigeon = new Pigeon2(deviceId, canBus);
    }

    @Override
    public void init() {
        // No-op for now. Device is already constructed. Implementations may
        // choose to factory-reset or configure here if desired.
    }

    @Override
    public Rotation2d getRotation2d() {
        return pigeon.getRotation2d();
    }

    @Override
    public double getPitch() {
        return pigeon.getPitch().getValueAsDouble();
    }

    @Override
    public double getRoll() {
        return pigeon.getRoll().getValueAsDouble();
    }

    @Override
    public double getAngularVelocityZWorld() {
        return pigeon.getAngularVelocityZWorld().getValueAsDouble();
    }
}
