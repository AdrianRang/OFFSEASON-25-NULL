// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ElevatorConstants.*;

public class Elevator extends SubsystemBase {
  public static enum  ElevatorPosition {
    ZERO(0.0),
    
		HOME(5.0),
    
		L1(HOME.getPosition()),

		L2(8.0), 
		L2_ALGAE(L2.getPosition()),

		L3(16),
		L3_ALGAE(L3.getPosition()),

		L4(56.0),

		STATION(5.0),

    INTAKE(2.5),
    NET(60);

		private double position;

		private ElevatorPosition(double position) {
			this.position = position;
		}

		public double getPosition() {
			return position;
		}
	}

  // (MASTER)
  private final SparkFlex leftMotor;
  private final SparkFlexConfig leftMotorConfig;

  private final RelativeEncoder leftEncoder;
  
  // (SLAVE)
  private final SparkFlex rightMotor;
  private final SparkFlexConfig rightMotorConfig;
  
  private ElevatorPosition setpoint = ElevatorPosition.ZERO; 
  private boolean pidEnabled = false;

  /** Creates a new Elevator. */
  public Elevator() {
    // * Left Motor (MASTER)
    this.leftMotor = new SparkFlex(leftMotorId, MotorType.kBrushless);
    this.leftMotorConfig = new SparkFlexConfig();
    this.leftMotorConfig
      .inverted(true)
      .idleMode(IdleMode.kBrake)
      .openLoopRampRate(kMotorRampRate)
      .closedLoopRampRate(kMotorRampRate)
      .smartCurrentLimit(kMototCurrentLimit)
      .voltageCompensation(12);
    
    // // TO DO: Encoder is not configured
    // ? Leave it like this?
    // * Yes
    
    this.leftMotor.configure(leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    this.leftEncoder = leftMotor.getEncoder();
    
    // (SLAVE)
    this.rightMotor = new SparkFlex(rightMotorId, MotorType.kBrushless);
    this.rightMotorConfig = new SparkFlexConfig();
    this.rightMotorConfig
    .idleMode(IdleMode.kBrake)
    .follow(leftMotorId, true) // Follows left motor (INVERTED)
    .openLoopRampRate(kMotorRampRate)
    .closedLoopRampRate(kMotorRampRate)
    .smartCurrentLimit(kMototCurrentLimit)
    .voltageCompensation(12);
    
    this.rightMotor.configure(rightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    // Log position setpoints for debugging
    for(ElevatorPosition pos : ElevatorPosition.values()) {
      SmartDashboard.putData("Elevator/Setpoint/" + pos.name(), setPostitionCommand(pos).ignoringDisable(true));
    }

    // Log PID for debugging
    SmartDashboard.putData("Elevator/PID", pidController);
  }

  
  public void setEncoder(double position) {
    leftEncoder.setPosition(position);
  }

  public void resetEncoder() {
    setEncoder(0.0);
  }

  public double getEncoderPosition() {
    return leftEncoder.getPosition();
  }

  public ElevatorPosition getSetpoint() {
    return setpoint;
  }

  public void setVoltage(double voltage) {
    pidEnabled = false;
    leftMotor.setVoltage(voltage);
  }

  //? Removed because we changed to ProfiledPidContoller instead of integrated spark encoder
  /*
   * Sets the goal position for the elevator
   * @param position The setpoint position of the elevator in meters
   */
  // private void setPosition(double position) {
  //   if(position > kMaxHeight || position < kMinHeight) return;
  //   pid.setReference(position, ControlType.kPosition);
  // }

  public void setSetpoint(ElevatorPosition position) {
    pidEnabled = true;
    this.setpoint = position;
  }

  public boolean isAtPosition() {
    return Math.abs(setpoint.getPosition() - getEncoderPosition()) < kPositionEpsilon;
  }

  ///// TO DO: Check if these RunCommands should be InstantCommands instead
  //? Instant command as you set the setpoint not drive the motor (motor is run on periodic) (no need to continuously run the command)
  public Command setPostitionCommand(ElevatorPosition position) {
    return new InstantCommand(()->setSetpoint(position), this);
  }

  public Command setPostitionWaitCommand(ElevatorPosition position) {
    return new InstantCommand(()->setSetpoint(position), this).until(this::isAtPosition);
  }

  public Command setVoltageCommand(double voltage) {
    return new RunCommand(() -> setVoltage(voltage), this);
  }

  public void resetPID() {
    pidController.reset(getEncoderPosition());
  }
 
  @Override
  public void periodic() {
    double pidResult = pidController.calculate(getEncoderPosition(), setpoint.getPosition());
    // ? setpoint should be passed instead of pid value to ff?
    double ffResult = feedforward.calculate(pidResult);

    if (pidEnabled) leftMotor.setVoltage(pidResult);
    // // TO DO: these 2 are the same
    SmartDashboard.putNumber("Elevator/RawPosition", leftEncoder.getPosition());
    SmartDashboard.putNumber("Elevator/Postition", getEncoderPosition());

    SmartDashboard.putBoolean("Elevator/IsAtPosition", isAtPosition());
    SmartDashboard.putNumber("Elevator/setpoint", setpoint.getPosition());

    SmartDashboard.putNumber("Elevator/LAppliedOutput", leftMotor.getAppliedOutput());
    SmartDashboard.putNumber("Elevator/RAppliedOutput", rightMotor.getAppliedOutput());

    SmartDashboard.putNumber("Elevator/PIDResult", pidResult);
    SmartDashboard.putNumber("Elevator/FFResult", ffResult);
  }
}
