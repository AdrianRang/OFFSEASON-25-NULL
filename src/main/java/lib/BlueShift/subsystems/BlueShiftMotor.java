// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package lib.BlueShift.subsystems;

import static edu.wpi.first.units.Units.Rotation;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BlueShiftMotor extends SubsystemBase {
  enum MotorType {
    NEO,
    NEO_VORTEX,
    KRAKEN_X60
  }
  public final MotorType type;

  TalonFX kraken;
  
  SparkFlex neo_vortex;

  SparkMax neo;

  /** Creates a new Kraken BlueShiftMotor
   * @param talon A `TalonFX` class previously configured
   */
  public BlueShiftMotor(TalonFX talon) {
    this.type = MotorType.KRAKEN_X60;
    this.kraken = talon;
  }

  /**
   * Creates a new Neo Vortex BlueShiftMotor.
   *
   * @param sparkFlex A `SparkFlex` class previously configured
   */
  public BlueShiftMotor(SparkFlex sparkFlex) {
    this.type = MotorType.NEO_VORTEX;
    this.neo_vortex = sparkFlex;
  }

  public BlueShiftMotor(SparkMax sparkMax) {
    this.type = MotorType.NEO;
    this.neo = sparkMax;
  }

  @Override
  public void periodic() {
    switch (type) {
      case NEO:
        SmartDashboard.putNumber("Motors/" + getName() + "/speed", neo.get());
        break;
      case NEO_VORTEX:
        SmartDashboard.putNumber("Motors/" + getName() + "/speed", neo_vortex.get());
        break;
      case KRAKEN_X60:
        SmartDashboard.putNumber("Motors/"+getName()+"/speed", kraken.get());
        SmartDashboard.putNumber("Motors/"+getName()+"/voltage", kraken.getMotorVoltage().getValueAsDouble());
        break;
      default:
        break;
    }

    SmartDashboard.putData("Motors/"+getName()+"/stop", setVolageCommand(0));
  }

  public void setVoltage(double voltage) {
    switch (type) {
      case NEO:
        neo.setVoltage(voltage);
        break;
      case NEO_VORTEX:
        neo_vortex.setVoltage(voltage);
        break;
      case KRAKEN_X60:
        kraken.setVoltage(voltage);
        break;
      default:
        throw new IllegalArgumentException("Invalid Motor Type");
    }
  }

  /**
   * @return the position according to the specified conversion factor (Neos) or rotations for krakens
   */
  public double getPosition() {
    switch (type) {
      case NEO:
        return neo.getEncoder().getPosition();
      case NEO_VORTEX:
        return neo_vortex.getEncoder().getPosition();
      case KRAKEN_X60:
        return kraken.getPosition().getValue().in(Rotation);
      default:
        throw new IllegalArgumentException("Invalid Motor Type");
    }
  }

  public Command setVolageCommand(double voltage) {
    return runOnce(()->setVoltage(voltage));
  }
}
