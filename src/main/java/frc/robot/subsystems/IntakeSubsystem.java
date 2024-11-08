// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DeviceIDs.CanIds;
import frc.robot.subsystems.BeamBreakSubsystem;

public class IntakeSubsystem extends SubsystemBase {
    private final SparkMax leftIntakeMotor;
    private final SparkMax rightIntakeMotor;
    private final SparkMax serializerBack;
    private SparkMaxConfig leftIntakeConfig;
    private SparkMaxConfig rightIntakeConfig;
    private SparkMaxConfig serializerBackConfig;

    private final BeamBreakSubsystem beamBreakSubsystem;
    private double baselineCurrentDraw;
    private static final double CURRENT_THRESHOLD = 17.0;  //needs to be adjusted by testing

    public IntakeSubsystem(BeamBreakSubsystem beamBreakSubsystem) {
        leftIntakeMotor = new SparkMax(CanIds.leftIntakeMotor.id, MotorType.kBrushless);
        rightIntakeMotor = new SparkMax(CanIds.rightIntakeMotor.id, MotorType.kBrushless);

        leftIntakeConfig = new SparkMaxConfig() {}; //not sure if this is the correct constructor initialization
        rightIntakeConfig = new SparkMaxConfig() {}; //not sure if this is the correct constructor initialization
        serializerBackConfig = new SparkMaxConfig() {}; //not sure if this is the correct constructor initialization

        this.beamBreakSubsystem = beamBreakSubsystem;
        serializerBack = new SparkMax(CanIds.serializerBack.id, MotorType.kBrushless);
        invertMotors();
        baselineCurrentDraw = serializerBack.getOutputCurrent();

        leftIntakeMotor.configure(leftIntakeConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        rightIntakeMotor.configure(rightIntakeConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        serializerBack.configure(serializerBackConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    public boolean frontBeamBreakIsTriggered() {
        return beamBreakSubsystem.frontBeamBreakIsTriggered();
    }

    public boolean backBeamBreakIsTriggered() {
        return !beamBreakSubsystem.backBeamBreakIsTriggered();
    }

    public void invertMotors() {
        serializerBack.setInverted(false);
        leftIntakeMotor.setInverted(true);
        rightIntakeMotor.setInverted(false);
    }

    //add voltage compensation and trapezoidal motion later
    public void setSpeed(double speed) {
        leftIntakeMotor.set(speed);
        rightIntakeMotor.set(speed);
    }

    public void setVoltage(double voltage) {
        leftIntakeMotor.setVoltage(voltage);
        rightIntakeMotor.setVoltage(voltage);
    }

    public void setSerializerVoltage(double voltage) {
        // serializerFront.setVoltage(voltage);
        serializerBack.setVoltage(voltage);
    }

    public void setSerializerSpeed(double speed) {
        // serializerFront.set(speed);
        serializerBack.set(speed);
    }

    public void brake() {
        leftIntakeConfig.idleMode(IdleMode.kBrake);
        rightIntakeConfig.idleMode(IdleMode.kBrake);

        leftIntakeMotor.configure(leftIntakeConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        rightIntakeMotor.configure(rightIntakeConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void brakeSerializer() {
        serializerBackConfig.idleMode(IdleMode.kBrake);
        serializerBack.configure(serializerBackConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void coast() {
        leftIntakeConfig.idleMode(IdleMode.kCoast);
        rightIntakeConfig.idleMode(IdleMode.kCoast);
        leftIntakeMotor.configure(leftIntakeConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        rightIntakeMotor.configure(rightIntakeConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void coastSerializer() {
        serializerBackConfig.idleMode(IdleMode.kCoast);
        serializerBack.configure(serializerBackConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        
    }

    public double getVelocity() {
        return leftIntakeMotor.get();
    }
    public boolean noteDetectedByCurrent() {
        double currentDraw = leftIntakeMotor.getOutputCurrent();
        return Math.abs(currentDraw - baselineCurrentDraw) > CURRENT_THRESHOLD;
    }

    public void updateBaselineCurrentDraw() {
        baselineCurrentDraw = leftIntakeMotor.getOutputCurrent();
    }
    @Override
    public void periodic() {
        SmartDashboard.putNumber("LeftIntakeSpeed", leftIntakeMotor.get());
        SmartDashboard.putNumber("RightIntakeSpeed", rightIntakeMotor.get());
        // SmartDashboard.putNumber("SerializerSpeed", serializerFront.get());
        SmartDashboard.putNumber("SerializerSpeed", serializerBack.get());
        SmartDashboard.putNumber("Current Draw", leftIntakeMotor.getOutputCurrent());

        SmartDashboard.putBoolean("Has Note", frontBeamBreakIsTriggered());
    }
}