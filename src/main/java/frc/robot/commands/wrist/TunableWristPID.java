package frc.robot.commands.wrist;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
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
 * Command to tune the Wrist PID and feedforward constants using SmartDashboard.
 * Allows real-time updates to PID constants and setpoints for wrist position
 * control.
 */
public class TunableWristPID extends Command {
  private final WristSubsystem wristSubsystem;
  private final ProfiledPIDController wristPidController;
  private final ArmFeedforward armFeedforward;

  /**
   * Constructs a new {@code TunableWristPID} command.
   * 
   * @param wristSubsystem the wrist subsystem used by this command
   */
  public TunableWristPID(WristSubsystem wristSubsystem) {
    this.wristSubsystem = wristSubsystem;

    // Initialize PID controller with SmartDashboard values or constants.
    wristPidController = new ProfiledPIDController(
        WristConstants.kP,
        WristConstants.kI,
        WristConstants.kD,
        new TrapezoidProfile.Constraints(
            WristConstants.maxVelocity.in(MetersPerSecond),
            WristConstants.maxAcceleration.in(MetersPerSecondPerSecond)));

    armFeedforward = new ArmFeedforward(WristConstants.kS, WristConstants.kV, WristConstants.kG);

    // Initialize SmartDashboard entries.
    SmartDashboard.putData("Wrist PID", wristPidController);
    SmartDashboard.putBoolean("Enable Wrist Tuning", false);

    addRequirements(wristSubsystem);
  }

  /**
   * Initializes the command by setting the wrist to coast mode and setting
   * the initial PID goal.
   */
  @Override
  public void initialize() {
    wristSubsystem.coast();
    wristPidController.setGoal(40);
    wristPidController.setTolerance(WristConstants.pidTolerance.in(Degrees));
  }

  /**
   * Executes the command by updating PID constants and setpoint if tuning is
   * enabled,
   * and applying the calculated output to the wrist motor.
   */
  @Override
  public void execute() {
    boolean tuningEnabled = SmartDashboard.getBoolean("Enable Wrist Tuning", false);

    if (tuningEnabled) {
      // Calculate PID and feedforward outputs.
      double output = wristPidController.calculate(wristSubsystem.getPosition().in(Degrees));
      Voltage feedforward = armFeedforward.calculate(
          wristSubsystem.getPosition(), wristSubsystem.getVelocity());

      wristSubsystem.set(output + feedforward.in(Volts));

      // Update SmartDashboard with telemetry.
      SmartDashboard.putNumber("Wrist Position", wristSubsystem.getPosition().in(Degrees));
      SmartDashboard.putNumber("Wrist Velocity", wristSubsystem.getVelocity().in(DegreesPerSecond));
      SmartDashboard.putNumber("Wrist PID Output", output);
    } else {
      wristSubsystem.set(0);
    }
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