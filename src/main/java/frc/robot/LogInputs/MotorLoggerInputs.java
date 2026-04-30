package frc.robot.LogInputs;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.hardware.traits.CommonTalon;
import com.revrobotics.spark.SparkMax;

@AutoLog
public class MotorLoggerInputs {
    public double voltage;
    public double velocity;
    public double outputCurrent;
    public double supplyCurrent;
    public double position;
    public double temperature;

    public void log(SparkMax motor) {
        voltage = motor.getAppliedOutput() * motor.getBusVoltage();
        outputCurrent = motor.getOutputCurrent();
        supplyCurrent = motor.getOutputCurrent();
        velocity = motor.getEncoder().getVelocity() / 60; // divide by 60 to convert from RPM to RPS
        position = motor.getEncoder().getPosition();
        temperature = motor.getMotorTemperature();
    }

    /**
     * this logger will record voltage, current, velocity, and position to our
     * Automatic Logger with any Talon motor
     * 
     * @param motor motor object (TalonFX, TalonFXS, etc.)
     */
    public void log(CommonTalon motor) {
        voltage = motor.getMotorVoltage().getValueAsDouble();
        outputCurrent = motor.getStatorCurrent().getValueAsDouble();
        supplyCurrent = motor.getSupplyCurrent().getValueAsDouble();
        velocity = motor.getVelocity().getValueAsDouble();
        position = motor.getPosition().getValueAsDouble();
        temperature = motor.getDeviceTemp().getValueAsDouble();
    }
}
