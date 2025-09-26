// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import static frc.robot.Constants.EndEffectorConstants.*;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.EndEffectorConstants.AlgeaConstants;
import frc.robot.Constants.EndEffectorConstants.CoralConstants;

public class EndEffector extends SubsystemBase {
  private final SparkMax coralMotor;
  private final SparkBaseConfig coralConfig;
  private final SparkMax algaeMotor;
  private final SparkBaseConfig algaeConfig;

  private final DigitalInput coralSwitch;

  /** Creates a new EndEffector. */
  public EndEffector() {
    coralMotor = new SparkMax(kCoralMotorID, MotorType.kBrushless);
    coralConfig = new SparkMaxConfig();
    coralConfig
      .voltageCompensation(12)
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(CoralConstants.currentLimit);

    algaeMotor = new SparkMax(kAlgaeMotorID, MotorType.kBrushless);
    algaeConfig = new SparkMaxConfig();
    algaeConfig
      .voltageCompensation(12)
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(AlgeaConstants.currentLimit);

    coralSwitch = new DigitalInput(kCoralSwitchID);

    coralMotor.configure(coralConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    algaeMotor.configure(algaeConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void intakeAlgae() {
    algaeMotor.set(AlgeaConstants.intakeSpeed);
  }

  public void holdAlgae() {
    algaeMotor.set(AlgeaConstants.holdSpeed);
  }

  public void outakeAlgae() {
    algaeMotor.set(AlgeaConstants.outakeSpeed);
  }

  public void stopAlgae() {
    algaeMotor.set(0);
  }

  public void intakeCoral() {
    coralMotor.set(CoralConstants.intakeSpeed);
  }

  public void outakeCoral() {
    coralMotor.set(CoralConstants.outakeSpeed);
  }

  public void stopCoral() {
    coralMotor.set(0);
  }

  public boolean hasCoral() {
    // return coralSwitch.get();
    return false;
  }

  public boolean hasAlgae() {
    // return algaeMotor.getOutputCurrent() > AlgeaConstants.checkCurrent;
    return false;
  }

  public Command intakeCoralCommand() {
    return new RunCommand(this::intakeCoral)
      .until(this::hasCoral)
      .andThen(this::stopCoral);
  }

  public Command outakeCoralCommand() {
    return new RunCommand(this::outakeCoral)
      .until(()->!hasCoral())
      .andThen(new WaitCommand(0.2))
      .andThen(this::stopCoral);
  }

  public Command intakeAlgaeCommand() {
    return new RunCommand(this::intakeAlgae)
      .until(this::hasAlgae)
      .andThen(this::holdAlgae);
  }

  public Command outakeAlgaeCommand() {
    return new RunCommand(this::outakeAlgae)
      .until(()->!hasAlgae())
      .andThen(new WaitCommand(0.2))
      .andThen(this::stopAlgae);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("EndEffector/hasCoral", hasCoral());
    SmartDashboard.putBoolean("EndEffector/hasAlgae", hasAlgae());
  }
}
