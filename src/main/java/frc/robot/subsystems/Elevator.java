// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ElevatorConstants.*;

public class Elevator extends SubsystemBase {
  public static enum  ElevatorPosition {
    // TODO:
		L1(0.5),

		L2(0.7), 
		L2_ALGAE_HIGH(L2.getPosition()), 
		L2_ALGAE_LOW(L2.getPosition()),

		L3(1),
		L3_ALGAE_HIGH(L3.getPosition()),
		L3_ALGAE_LOW(L3.getPosition()),

		L4(1.4),

		SOURCE(0.5),

		HOME(0.2),

    INTAKE(0.3),
    NET(2.4);

		private double position;

		private ElevatorPosition(double position) {
			this.position = position;
		}

		public double getPosition() {
			return position;
		}
	}

  private final SparkFlex leftMotor;
  private final SparkFlex rightMotor;
  private final SparkFlexConfig leftMotorConfig;
  private final SparkFlexConfig rightMotorConfig;

  private final RelativeEncoder encoder;
  private final SparkClosedLoopController pid;

  private double setpoint; 

  /** Creates a new Elevator. */
  public Elevator() {
    this.leftMotor = new SparkFlex(leftMotorId, MotorType.kBrushless);
    this.rightMotor = new SparkFlex(rightMotorId, MotorType.kBrushless);

    this.leftMotorConfig = new SparkFlexConfig();
    this.leftMotorConfig
      .idleMode(IdleMode.kBrake)
      .openLoopRampRate(kMotorRampRate)
      .closedLoopRampRate(kMotorRampRate)
      .smartCurrentLimit(kMototCurrentLimit);
    
    this.rightMotorConfig = new SparkFlexConfig();
    this.rightMotorConfig
      .idleMode(IdleMode.kBrake)
      .follow(leftMotorId, true)
      .openLoopRampRate(kMotorRampRate)
      .closedLoopRampRate(kMotorRampRate)
      .smartCurrentLimit(kMototCurrentLimit);
    
    this.leftMotorConfig.closedLoop
      .p(kP)
      .i(kI)
      .d(kD);
    
    this.leftMotor.configure(leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    this.rightMotor.configure(rightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    this.encoder = leftMotor.getEncoder();

    this.pid = leftMotor.getClosedLoopController();
  }

  public void resetEncoder() { encoder.setPosition(0); }

  public void setEncoder(double position) { encoder.setPosition(position); }

  /**
   * Gets the position of the elevator
   * @return The position in meters
   */
  public double getPosition() {
    return encoder.getPosition() * kRotationToHeightRatio;
  }

  /**
   * Sets the goal position for the elevator
   * @param position The setpoint position of the elevator in meters
   */
  public void setPosition(double position) {
    if(position > kMaxHeight || position < kMinHeight) return;

    setpoint = position;
    pid.setReference(position/kRotationToHeightRatio, ControlType.kPosition);
  }

  /**
   * @return True if the position is within epsilon of the setpoint
   */
  public boolean isAtPosition() {
    return Math.abs(getPosition() - encoder.getPosition() * kRotationToHeightRatio) < kPositionEpsilon;
  }

  public Command setPostitionCommand(double position) {
    return new RunCommand(()->setPosition(position), this);
  }

  public void stop() {
    leftMotor.set(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator/RawPosition", encoder.getPosition());
    SmartDashboard.putNumber("Elevator/Postition", getPosition());
    SmartDashboard.putBoolean("Elevator/IsAtPosition", isAtPosition());
    SmartDashboard.putNumber("Elevator/setpoint", setpoint);
  }
}
