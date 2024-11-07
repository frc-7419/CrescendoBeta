// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wrist;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DeviceIDs.CanIds;
import frc.robot.constants.DeviceIDs.SensorIds;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.RobotConstants.ShooterConstants;

public class WristSubsystem extends SubsystemBase {
  private final TalonFX wristMotor;
  private final DutyCycleEncoder wristEncoder;

  /** Creates a new ArmSubsystem. */
  public WristSubsystem() {
    wristMotor = new TalonFX(CanIds.shooterWrist.id, RobotConstants.canBus);
    wristEncoder = new DutyCycleEncoder(SensorIds.wristEncoder.id);
  }

  public void set(double voltage) {
    wristMotor.setVoltage(voltage);
  }

  public void brake() {
    wristMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  public Angle getPosition() {
    return Rotations
        .of(wristEncoder.get() + ShooterConstants.encoderOffset.in(Rotations)
            + ShooterConstants.wristOffset.in(Rotations));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Wrist Angle", getPosition().in(Degrees));
    SmartDashboard.putNumber("Wrist Encoder Value", getPosition().in(Rotations));
    SmartDashboard.putNumber("Wrist Raw Encoder Value", getPosition().in(Rotations) - ShooterConstants.wristOffset
        .in(Rotations));
  }
}