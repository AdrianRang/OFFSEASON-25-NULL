// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.Constants.ArmConstants.*;
import static frc.robot.Constants.ElevatorConstants.kPositionEpsilon;

import java.util.function.Supplier;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Elevator.ElevatorPosition;

public class Arm extends SubsystemBase {
  public static enum ArmPosition {
    INTAKE(180),
    IDLE(150),
    LLOW(40),
    L4(90),
    NET(10);

    private double position;
    private ArmPosition(double position) {
      this.position = position;
    }

    public double getPosition() {return position;}
  }

  private final SparkFlex motor;
  private final SparkFlexConfig motorConfig;

  private final SparkClosedLoopController pid;
  private final RelativeEncoder encoder;

  private final CANcoder absoluteEncoder;

  private final Supplier<Elevator.ElevatorPosition> elevatorPosition;

  private ArmPosition currentPosition = ArmPosition.IDLE;

  /** Creates a new Arm. */
  public Arm(Supplier<Elevator.ElevatorPosition> positionSupplier) {
    motor = new SparkFlex(kMotorId, MotorType.kBrushless);
    motorConfig = new SparkFlexConfig();
    motorConfig
      .idleMode(IdleMode.kBrake)
      .openLoopRampRate(kMotorRampRate)
      .closedLoopRampRate(kMotorRampRate)
      .smartCurrentLimit(kMotorCurrentLimit)
      .closedLoop.pid(kP, kI, kD);
    motorConfig.encoder.positionConversionFactor(kConversionFactor);
    motor.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    absoluteEncoder = new CANcoder(kAbsoluteEncoderId);

    elevatorPosition = positionSupplier;

    encoder = motor.getEncoder();
    pid = motor.getClosedLoopController();

    encoder.setPosition(getAbsolutePosition().in(Rotations));
  }

  private void setPosition(double position) {
    if(position > kMax || position < kMin) return;
    pid.setReference(position, ControlType.kPosition);
  }

  public Angle getAbsolutePosition() {
    return absoluteEncoder.getAbsolutePosition().refresh().getValue();
  }

  public void setPosition(ArmPosition position) {
    // TODO: Check the actual encoder value, instead of the setpoint position
    // TODO: Schedule command after the elevator position goes below a certain threshold
    if(elevatorPosition.get() == ElevatorPosition.INTAKE && position == ArmPosition.INTAKE) return;
    currentPosition = position;
    setPosition(position.getPosition());
  }

  public boolean atPosition() {
    return Math.abs(encoder.getPosition() - currentPosition.getPosition()) < kPositionEpsilon;
  }

  public boolean atPosition(ArmPosition position) {
    return Math.abs(encoder.getPosition() - position.getPosition()) < kPositionEpsilon;
  }

  public Command setPostionCommand(ArmPosition position) {
    return new InstantCommand(()->setPosition(position), this);
  }

  public Command setPostionWaitCommand(ArmPosition position) {
    return new InstantCommand(()->setPosition(position), this).until(this::atPosition);
  }

  @Override
  public void periodic() {
    if(elevatorPosition.get() != ElevatorPosition.INTAKE && currentPosition == ArmPosition.INTAKE) setPosition(ArmPosition.IDLE);

    SmartDashboard.putNumber("Arm/MotEncPositionDeg", encoder.getPosition() * 360);
    SmartDashboard.putNumber("Arm/AbsEncPosition", getAbsolutePosition().in(Degrees));
  }
}
