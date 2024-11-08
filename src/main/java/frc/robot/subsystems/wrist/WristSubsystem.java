// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wrist;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DeviceIDs.CanIds;
import frc.robot.constants.DeviceIDs.SensorIds;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.RobotConstants.WristConstants;

/**
 * Subsystem for controlling the wrist mechanism of the robot.
 */
public class WristSubsystem extends SubsystemBase {
  private final TalonFX wristMotor;
  private final DutyCycleEncoder wristEncoder;
  private final Alert noEncoder = new Alert("Shooter Wrist encoder not found on DIO " + SensorIds.wristEncoder.id,
      AlertType.kError);

  /**
   * Creates a new WristSubsystem.
   * Initializes the motor and encoder for the wrist.
   */
  public WristSubsystem() {
    wristMotor = new TalonFX(CanIds.shooterWrist.id, RobotConstants.canBus);
    wristEncoder = new DutyCycleEncoder(SensorIds.wristEncoder.id);
  }

  /**
   * Sets the motor output to a specific voltage.
   * 
   * @param voltage the voltage to apply to the motor
   */
  public void set(double voltage) {
    Angle currentPosition = getPosition();

    // Prevent movement beyond max limit
    if (currentPosition.gt(WristConstants.maxLimit) && voltage > 0) {
      wristMotor.setVoltage(0);
      return;
    }

    // Prevent movement below min limit
    if (currentPosition.lt(WristConstants.minLimit) && voltage < 0) {
      wristMotor.setVoltage(0);
      return;
    }

    // Apply voltage if within limits
    wristMotor.setVoltage(voltage);
  }

  /**
   * Sets the motor to coast mode.
   * This allows the motor to freely rotate when no power is applied.
   */
  public void coast() {
    wristMotor.setNeutralMode(NeutralModeValue.Coast);
  }

  /**
   * Sets the motor to brake mode.
   * This stops the motor from freely rotating when no power is applied.
   */
  public void brake() {
    wristMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  /**
   * Gets the current position of the wrist in rotations.
   * 
   * @return the wrist's current angle as an {@link Angle}
   */
  public Angle getPosition() {
    return Rotations
        .of(wristEncoder.get() + WristConstants.encoderOffset.in(Rotations)
            + WristConstants.wristOffset.in(Rotations));
  }

  /**
   * Gets the current velocity of the wrist in rotations per second.
   * 
   * @return the wrist's current angular velocity as an {@link AngularVelocity}
   */
  public AngularVelocity getVelocity() {
    return wristMotor.getVelocity().getValue().times(WristConstants.gearRatio);
  }

  /**
   * This method is called periodically and updates the SmartDashboard with
   * wrist subsystem data.
   */
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Wrist Angle", getPosition().in(Degrees));
    SmartDashboard.putNumber("Wrist Velocity", getVelocity().in(RotationsPerSecond));
    SmartDashboard.putNumber("Wrist Encoder Value", getPosition().in(Rotations));
    SmartDashboard.putNumber("Wrist Raw Encoder Value", getPosition().in(Rotations) - WristConstants.wristOffset
        .in(Rotations));
    if (wristEncoder.isConnected()) {
      noEncoder.set(true);
    } else {
      noEncoder.set(false);
    }
  }
}