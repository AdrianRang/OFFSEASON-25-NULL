// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.Constants.IntakeConstants.PivotConstants.*;

public class IntaxerPivot extends SubsystemBase {
  private final TalonFX motor;
  private final TalonFXConfiguration motorConfig;
  private final CANcoder encoder;

  private double setpoint = 0;

  public IntaxerPivot() {
    motor = new TalonFX(kMotorId);

    motorConfig = new TalonFXConfiguration();
    motorConfig
      .withCurrentLimits(
        new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(kMotorCurrentLimit)
            .withSupplyCurrentLowerLimit(kMotorLowerCurrentLimit)
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLowerTime(0.2)
        )
        .withOpenLoopRamps(
            new OpenLoopRampsConfigs()
                .withVoltageOpenLoopRampPeriod(kMotorRampRate)
        )
        .withVoltage(
            new VoltageConfigs()
                .withPeakForwardVoltage(12)
                .withPeakReverseVoltage(-12)
        )
        .withMotorOutput(
            new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.Clockwise_Positive)
        );

    motor.getConfigurator().apply(motorConfig);

    encoder = new CANcoder(kEncoderId);

    SmartDashboard.putData("IntaxerPivot/PID", pid);
    // SmartDashboard.putData("IntaxerPivot/resetPos", resetPosition());
    // SmartDashboard.putData("IntaxerPivot/resetDown", resetDown());
  }

  public double getPosition() {
    return encoder.getPosition().refresh().getValueAsDouble();
  }

  public void setSetpoint(double setpoint) {
    pid.reset(getPosition());
    this.setpoint = setpoint;
  }

  public void setUp() {
    setSetpoint(kUpPos);
  }

  public void setDown() {
    setSetpoint(kDownPos);
  }

  // public Command resetPosition() {
  //   return new InstantCommand(()->motor.setPosition(Degrees.of(0))).ignoringDisable(true);
  // }

  // public Command resetDown() {
  //   return new InstantCommand(()->motor.setPosition(Rotations.of(kDownPos))).ignoringDisable(true);
  // }

  public Command setUpCommand() {return new InstantCommand(this::setUp, this);}
  public Command setDownCommand() {return new InstantCommand(this::setDown, this);}

  public double getSetpoint() {
    return setpoint;
  }

  boolean activeLast = false;

  @Override
  public void periodic() {
    double pidResult = pid.calculate(getPosition(), setpoint);
    double ffResult = feedforward.calculate(Rotations.of(getPosition() + 0.25).in(Radians), pidResult);
    SmartDashboard.putNumber("IntaxerPivot/PIDResult", pidResult);
    SmartDashboard.putNumber("IntaxerPivot/FFResult", ffResult);
    if(Math.abs(getPosition() - getSetpoint()) > kEpsilon) {motor.setVoltage(ffResult); if(!activeLast) pid.reset(getPosition()); activeLast = true;} else activeLast = false;

    SmartDashboard.putNumber("IntaxerPivot/Setpoint", setpoint);
    SmartDashboard.putNumber("IntaxerPivot/Position", getPosition());
  }
}
