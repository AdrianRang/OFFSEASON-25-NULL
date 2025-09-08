// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package lib.BlueShift.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Servo extends SubsystemBase {

  private boolean enabled = false;
  private final PIDController pidController;
  private Supplier<Double> measurementSupplier;
  private double setpoint;
  private final BlueShiftMotor[] motors;

  /**
   * Creates a new Servo class
   * @param pid the pid controller to control the motors
   * @param motors The motors, be sure to invert nexessary inversions in the motors config
   * The measurement is set to the {@link BlueShiftMotor#getPosition()} of the first motor
   */
  public Servo(PIDController pid, BlueShiftMotor... motors) {
    this.motors = motors;
    this.pidController = pid;
  }

  public Servo(PIDController pid, Supplier<Double> measurementSupplier, BlueShiftMotor... motors) {
    this(pid, motors);
    this.measurementSupplier = measurementSupplier;
  }

  /**
   * Creates a new Servo class
   * @param pid the pid controller to control the motors
   * @param motors The motors, be sure to invert nexessary inversions in the motors config
   * The measurement is set to the {@link BlueShiftMotor#getPosition()} of the first motor
   */
  public Servo(double p, double i, double d, BlueShiftMotor... motors) {
    this.motors = motors;
    this.pidController = new PIDController(p, i, d);
    this.measurementSupplier = motors[0]::getPosition;
  }

  public Servo(double p, double i, double d, Supplier<Double> measurementSupplier, BlueShiftMotor... motors) {
    this(p, i, d , motors);
    this.measurementSupplier = measurementSupplier;
  }
  public void enable() {enabled = true;}
  public void disable() {enabled = false;}

  public void setMeasurementSupplier(Supplier<Double> supplier) {
    this.measurementSupplier = supplier;
  }

  @Override
  public void periodic() {
    if(enabled) {
      for(BlueShiftMotor motor : motors) {
        motor.setVoltage(pidController.calculate(measurementSupplier.get(), setpoint));
      }
    } else {
      for(BlueShiftMotor motor : motors) {
        motor.setVoltage(pidController.calculate(0));
      }
    }

    SmartDashboard.putNumber("Servos/"+getName()+"/pidOutput", pidController.calculate(measurementSupplier.get(), setpoint));
  }
}
