// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.Constants.ArmConstants.*;
import static frc.robot.Constants.ElevatorConstants.kPositionEpsilon;
import java.util.function.Supplier;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase; 

public class Arm extends SubsystemBase {
  public static enum ArmPosition {
    INTAKE(Degrees.of(220)),
    IDLE(Degrees.of(180)),
    PLACE(Degrees.of(37)),
    NET(Degrees.of(37));

    private Angle position;
    private ArmPosition(Angle position) {
      this.position = position;
    }

    public Angle getPosition() {return position;}
  }

  // Motor
  private final SparkFlex motor;
  private final SparkFlexConfig motorConfig;

  // Encoder
  private final RelativeEncoder encoder;

  // Absolute Encoder
  private final CANcoder absoluteEncoder;

  // Setpoint
  private ArmPosition setpoint = ArmPosition.IDLE;

  /** Creates a new Arm. */
  public Arm(Supplier<Elevator.ElevatorPosition> positionSupplier) {
    // Motor
    motor = new SparkFlex(kMotorId, MotorType.kBrushless);
    motorConfig = new SparkFlexConfig();
    motorConfig
      .idleMode(IdleMode.kBrake)
      .openLoopRampRate(kMotorRampRate)
      .closedLoopRampRate(kMotorRampRate)
      .smartCurrentLimit(kMotorCurrentLimit)
      .voltageCompensation(12);
    motorConfig.encoder
      .positionConversionFactor(kConversionFactor);
    motor.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    // Encoder
    encoder = motor.getEncoder();

    // Absolute Encoder
    absoluteEncoder = new CANcoder(kAbsoluteEncoderId);

    // Reset encoder
    encoder.setPosition(getAbsolutePosition().in(Rotations));

    // Log position setpoints for debugging
    for(ArmPosition pos : ArmPosition.values()) {
      SmartDashboard.putData("Arm/Setpoint/" + pos.name(), setPostionCommand(pos).ignoringDisable(true));
    }
  }

  // private void setPosition(Angle position) {
  //   if(position.in(Rotations) > kMax.in(Rotations) || position.in(Rotations) < kMin.in(Rotations)) {
  //     System.out.println("Arm angle overshoot");
  //   };
  //   pid.setReference(position.in(Rotations), ControlType.kMAXMotionPositionControl);
  // }

  public void setPosition(ArmPosition position) {
    setpoint = position;
  }

  public Angle getAbsolutePosition() {
    return absoluteEncoder.getAbsolutePosition().refresh().getValue();
  }


  public boolean atPosition() {
    return Math.abs(encoder.getPosition() - setpoint.getPosition().in(Rotations)) < kPositionEpsilon;
  }

  public Command setPostionCommand(ArmPosition position) {
    return new InstantCommand(()->setPosition(position), this);

  }
  public Command setPostionWaitCommand(ArmPosition position) {
    return new InstantCommand(()->setPosition(position), this).until(this::atPosition);
  }

  @Override
  public void periodic() {
    // TODO: Try with absolute encoder instead of integrated encoder
    double pidResult = pidController.calculate(encoder.getPosition(), setpoint.getPosition().in(Rotations));
    // ? setpoint should be passed instead of pid value to ff?
    double ffResult = feedforward.calculate(Rotations.of(encoder.getPosition()).in(Radians), pidResult);

    // ! DON'T APPLY VOLTAGE TO MOTOR BEFORE TESTING
    motor.setVoltage(pidResult);

    SmartDashboard.putNumber("Arm/MotEncPositionDeg", encoder.getPosition() * 360);
    SmartDashboard.putNumber("Arm/AbsEncPosition", getAbsolutePosition().in(Degrees));
    SmartDashboard.putNumber("Arm/SetpointPosition", setpoint.getPosition().in(Degrees));
    SmartDashboard.putBoolean("Arm/AtPosition", atPosition());
    SmartDashboard.putNumber("Arm/AppliedOutput", motor.getAppliedOutput());
    SmartDashboard.putNumber("Arm/PIDResult", pidResult);
    SmartDashboard.putNumber("Arm/FFResult", ffResult);
  }
}
