// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANrange;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import static frc.robot.Constants.ArmConstants.kMotorCurrentLimit;
import static frc.robot.Constants.IntakeConstants.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class Intakexer extends SubsystemBase {
  private final SparkMax intakeMotor;
  private final SparkMaxConfig intakeMotorConfig;
  private final SparkMax indexMotor;
  private final SparkMaxConfig indexMotorConfig;
  private final SparkMax passMotor;
  private final SparkMaxConfig passMotorConfig;

  private final CANrange checkSensor;

  public Intakexer() {
    intakeMotor = new SparkMax(kIntakeMotorId, MotorType.kBrushless);
    intakeMotorConfig = new SparkMaxConfig();
    intakeMotorConfig
      .smartCurrentLimit(kMotorCurrentLimit)
      .idleMode(IdleMode.kCoast);
    intakeMotor.configure(intakeMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    indexMotor = new SparkMax(kIndexerMotorId, MotorType.kBrushless);
    indexMotorConfig = new SparkMaxConfig();
    indexMotorConfig
      .smartCurrentLimit(kMotorCurrentLimit)
      .idleMode(IdleMode.kCoast);
    indexMotor.configure(indexMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    passMotor = new SparkMax(kPassMotorId, MotorType.kBrushless);
    passMotorConfig = new SparkMaxConfig();
    passMotorConfig
      .smartCurrentLimit(kMotorCurrentLimit)
      .idleMode(IdleMode.kBrake);
    passMotor.configure(passMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    checkSensor = new CANrange(kCANrangeId);

    SmartDashboard.putData("Intake/stop", stopCommand());
    SmartDashboard.putData("Intake/eject", ejectCommand());
  }

  public void startIntake() {
    intakeMotor.set(kIntakeSpeed);
    indexMotor.set(kIndexSpeed);
    passMotor.set(kPassSpeed);
  }

  private void passToEndEffector() {
    passMotor.set(kPassSpeed);
  }

  private void eject() {
    intakeMotor.set(-kEjectSpeed);
    indexMotor.set(kEjectSpeed);
    passMotor.set(kEjectSpeed);
  }

  public void stop() {
    intakeMotor.set(0);
    indexMotor.set(0);
    passMotor.set(0);
  }

  public boolean hasCoral() {
    return checkSensor.getDistance().getValueAsDouble() < kCheckRange;
  }

  public Command passCommand() {
    return new RunCommand(this::passToEndEffector, this).until(()->!hasCoral());
  }

  public Command intakeCommand() {
    return new InstantCommand(this::startIntake, this);
  }

  public Command ejectCommand() {
    return new InstantCommand(this::eject, this);
  }

  public Command stopCommand() {
    return new InstantCommand(this::stop, this);
  }

  public Command intakeWaitCommand() {
    return intakeCommand()
      .andThen(new WaitUntilCommand(this::hasCoral))
      .andThen(stopCommand());
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Intake/hasCoral", hasCoral());
  }
}
