// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.ClosedLoopSlot;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DeviceIDs.CanIds;
import frc.robot.constants.RobotConstants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
    /**
     * Creates a new Shooter.
     */
    private final SparkFlex shooterMotorTop;
    private final SparkFlex shooterMotorBottom;
    private final RelativeEncoder topShooterEncoder;
    private final RelativeEncoder bottomShooterEncoder;

    private final SparkBaseConfig sparkBaseConfigTop;
    private final SparkBaseConfig sparkBaseConfigBottom;

    private final SparkClosedLoopController topShooterPidController;
    private final ClosedLoopConfig topShooterPidConfig;
    private final SimpleMotorFeedforward topFeedforward = new SimpleMotorFeedforward(0.10894, 0.10806, 0.015777);

    private final SparkClosedLoopController bottomShooterPidController;
    private final ClosedLoopConfig bottomShooterPidConfig;
    private final SimpleMotorFeedforward bottomFeedforward = new SimpleMotorFeedforward(0.10894, 0.10806, 0.015777);

    private boolean isRunning;
    private double topPIDsetpoint;
    private double bottomPIDsetpoint;
   

    public ShooterSubsystem() {
        shooterMotorTop = new SparkFlex(CanIds.topShooter.id, MotorType.kBrushless);
        shooterMotorBottom = new SparkFlex(CanIds.bottomShooter.id, MotorType.kBrushless);
        topShooterEncoder = shooterMotorTop.getEncoder();
        bottomShooterEncoder = shooterMotorBottom.getEncoder();

        sparkBaseConfigTop = new SparkBaseConfig() {}; // not sure if this is the correct constructor declaration
        sparkBaseConfigBottom = new SparkBaseConfig() {}; // not sure if this is the correct constructor declaration
        
        invertMotors();
        isRunning = true; 
        topShooterPidController = shooterMotorTop.getClosedLoopController();
        topShooterPidController.setReference(topPIDsetpoint, ControlType.kVelocity, 0);
        topShooterPidConfig = new ClosedLoopConfig();
        topShooterPidConfig.p(0.00065 * 2, ClosedLoopSlot.kSlot0);
        topShooterPidConfig.i(0, ClosedLoopSlot.kSlot0);
        topShooterPidConfig.d(0, ClosedLoopSlot.kSlot0);
        topShooterPidConfig.iZone(0, ClosedLoopSlot.kSlot0);
        topShooterPidConfig.outputRange(-1, 1, ClosedLoopSlot.kSlot0);
        sparkBaseConfigTop.closedLoop.apply(topShooterPidConfig);
        sparkBaseConfigTop.smartCurrentLimit(ShooterConstants.topShooterStallLimit, ShooterConstants.topShooterFreeLimit);
        shooterMotorTop.configure(sparkBaseConfigTop, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        bottomShooterPidController = shooterMotorBottom.getClosedLoopController();
        bottomShooterPidController.setReference(bottomPIDsetpoint, ControlType.kVelocity, 0);
        bottomShooterPidConfig = new ClosedLoopConfig();
        bottomShooterPidConfig.p(0.00065 * 2, ClosedLoopSlot.kSlot0);
        bottomShooterPidConfig.i(0, ClosedLoopSlot.kSlot0);
        bottomShooterPidConfig.d(0, ClosedLoopSlot.kSlot0);
        bottomShooterPidConfig.iZone(0, ClosedLoopSlot.kSlot0);
        bottomShooterPidConfig.outputRange(-1, 1, ClosedLoopSlot.kSlot0);
        sparkBaseConfigBottom.closedLoop.apply(bottomShooterPidConfig);
        sparkBaseConfigBottom.smartCurrentLimit(ShooterConstants.bottomShooterStallLimit, ShooterConstants.bottomShooterFreeLimit);
        shooterMotorBottom.configure(sparkBaseConfigBottom, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setRPM(double topRPM, double bottomRPM) {
        // topShooterPidController.setFF(0.0003 / 2);
        // bottomShooterPidController.setFF(0.0003 / 2);

        topShooterPidController.setReference(bottomPIDsetpoint, null, 0, 0.0003/2);
        bottomShooterPidController.setReference(bottomPIDsetpoint, null, 0, 0.0003/2);

        // System.out.println("changing rpm" + topRPM + bottomRPM);
        topShooterPidController.setReference(topRPM, ControlType.kVelocity);
        bottomShooterPidController.setReference(bottomRPM, ControlType.kVelocity);
    }

    public void invertMotors() {
        shooterMotorBottom.setInverted(true);
        shooterMotorTop.setInverted(false);
    }

    public void setTopSpeed(double speed) {
        shooterMotorTop.set(speed);
    }

    public void setBottomSpeed(double speed) {
        shooterMotorBottom.set(speed);
    }

    public void setBothSpeed(double speed) {
        setTopSpeed(speed);
        setBottomSpeed(speed);
    }

    public double getTopPIDsetpoint() {
        return this.topPIDsetpoint;
    }

    public void setTopPIDsetpoint(double setpoint) {
        this.topPIDsetpoint = setpoint;
    }

    public double getBottomPIDsetpoint() {
        return this.bottomPIDsetpoint;
    }

    public void setBottomPIDsetpoint(double setpoint) {
        this.bottomPIDsetpoint = setpoint;
    }

    public void setTopVoltage(double voltage) {
        shooterMotorTop.setVoltage(voltage);
    }

    public void setBottomVoltage(double voltage) {
        shooterMotorBottom.setVoltage(voltage);
    }

    public void setBothVoltage(double voltage) {
        setTopVoltage(voltage);
        setBottomVoltage(voltage);
    }

    public double getTopVelocity() {
        return shooterMotorTop.getEncoder().getVelocity();
    }

    public double getBottomVelocity() {
        return shooterMotorBottom.getEncoder().getVelocity();
    }

    public void brake() {
        // shooterMotorTop.stopMotor();
        // shooterMotorBottom.stopMotor();

        sparkBaseConfigTop.idleMode(SparkBaseConfig.IdleMode.kBrake);
        sparkBaseConfigBottom.idleMode(SparkBaseConfig.IdleMode.kBrake);

        shooterMotorTop.configure(sparkBaseConfigTop, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        shooterMotorBottom.configure(sparkBaseConfigTop, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    }

    public void coast() {

        sparkBaseConfigTop.idleMode(SparkBaseConfig.IdleMode.kCoast);
        sparkBaseConfigBottom.idleMode(SparkBaseConfig.IdleMode.kCoast);

        shooterMotorTop.configure(sparkBaseConfigTop, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        shooterMotorBottom.configure(sparkBaseConfigTop, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    }

    public boolean getToggle() {
        return isRunning;
    }

    public void invertToggle() {
        this.isRunning = !this.isRunning;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        SmartDashboard.putNumber("Top Shooter Velocity", getTopVelocity());
        SmartDashboard.putNumber("Bottom Shooter Velocity", getBottomVelocity());
        SmartDashboard.putBoolean("shooterToggle", isRunning);
    }
}
// 47