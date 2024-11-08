// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.wrist;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.RobotConstants.WristConstants;
import frc.robot.subsystems.wrist.WristSubsystem;

/**
 * Command to move the wrist to a specific position using a Profiled PID
 * Controller and
 * Feedforward
 */
public class WristToPosition extends Command {
  private final ArmFeedforward armFeedforward;
  private final ProfiledPIDController wristPidController;
  private final double angle;
  private final WristSubsystem wristSubsystem;

  /**
   * Constructs a new {@code WristToPosition} command.
   * 
   * @param wristSubsystem the wrist subsystem used by this command
   * @param angle          the target angle for the wrist, in degrees
   */
  public WristToPosition(WristSubsystem wristSubsystem, double angle) {
    wristPidController = new ProfiledPIDController(WristConstants.kP, WristConstants.kI, WristConstants.kD,
        new TrapezoidProfile.Constraints(WristConstants.maxVelocity.in(MetersPerSecond),
            WristConstants.maxAcceleration.in(MetersPerSecondPerSecond)));
    armFeedforward = new ArmFeedforward(WristConstants.kS, WristConstants.kV, WristConstants.kG);
    this.wristSubsystem = wristSubsystem;
    this.angle = angle;
    addRequirements(wristSubsystem);
  }

  /**
   * Initializes the command by setting the motor to coast mode and resetting the
   * PID controller.
   */
  @Override
  public void initialize() {
    wristSubsystem.coast();
    wristPidController.setGoal(angle);

    wristSubsystem.set(0);
    wristPidController.setTolerance(WristConstants.pidTolerance.in(Degrees));

    SmartDashboard.putNumber("Wrist Setpoint", angle);
  }

  /**
   * Executes the command by calculating the PID and feedforward output and
   * applying
   * the result to the wrist motor.
   */
  @Override
  public void execute() {
    double output = wristPidController.calculate(wristSubsystem.getPosition().in(Degrees));
    Voltage feedforward = armFeedforward.calculate(wristSubsystem.getPosition(), wristSubsystem.getVelocity());
    wristSubsystem.set(output + feedforward.in(Volts));
    SmartDashboard.putNumber("Wrist PID Output", output);
    SmartDashboard.putBoolean("At Goal", wristPidController.atGoal());
    SmartDashboard.putData(wristPidController);
  }

  /**
   * Ends the command by stopping the wrist motor.
   * 
   * @param interrupted whether the command was interrupted
   */
  @Override
  public void end(boolean interrupted) {
    wristSubsystem.set(0);
  }

  /**
   * Determines whether the command has finished.
   * 
   * @return always returns {@code false}, as this command runs until interrupted
   */
  @Override
  public boolean isFinished() {
    return false;
  }
}