// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DeviceIDs.CanIds;
import frc.robot.constants.DeviceIDs.SensorIds;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.RobotConstants.ShooterConstants;

public class ArmSubsystem extends SubsystemBase {
  private final TalonFX armMotor;
  private final DutyCycleEncoder armEncoder;
  private final ArmFeedforward armFeedforward;
  private final PIDController pidController;

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    armMotor = new TalonFX(CanIds.shooterWrist.id, RobotConstants.canBus);
    armEncoder = new DutyCycleEncoder(SensorIds.armEncoder.id);
    armFeedforward = new ArmFeedforward(0, 0, 0);
    pidController = new PIDController(0, 0, 0);
    setGains();
  }

  public void setGains() {

  }

  public void coast() {
    armMotor.setNeutralMode(NeutralModeValue.Coast);
  }

  public void brake() {
    armMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  public Angle getPosition() {
    return Rotations
        .of(armEncoder.get() + ShooterConstants.encoderOffset.in(Rotations) + ShooterConstants.armOffset.in(Rotations));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm Angle", getPosition().in(Degrees));
    SmartDashboard.putNumber("Arm Encoder Value", getPosition().in(Rotations));
    SmartDashboard.putNumber("Arm Raw Encoder Value", getPosition().in(Rotations) - ShooterConstants.armOffset
        .in(Rotations));
    SmartDashboard.putData("Arm PID", pidController);

  }
}
