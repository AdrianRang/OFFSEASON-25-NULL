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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.EndEffectorConstants.AlgeaConstants;

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
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(CoralConstants.currentLimit);

    algaeMotor = new SparkMax(kAlgeaMotorID, MotorType.kBrushless);
    algaeConfig = new SparkMaxConfig();
    algaeConfig
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(AlgeaConstants.currentLimit);

    coralSwitch = new DigitalInput(kCoralSwitchID);

    coralMotor.configure(coralConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    algaeMotor.configure(algaeConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void intakeAlgae() {
    algaeMotor.set(AlgeaConstants.intakeSpeed);
  }

  public void holdAlgea() {
    algaeMotor.set(AlgeaConstants.holdSpeed);
  }

  public void outakeAlgae() {
    algaeMotor.set(AlgeaConstants.outakeSpeed);
  }

  public void stopAlgea() {
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

  public boolean getCoral() {
    return coralSwitch.get();
  }

  public boolean getAlgae() {
    return algaeMotor.getOutputCurrent() > AlgeaConstants.checkCurrent;
  }

  public Command intakeCoralCommand() {
    return new InstantCommand(this::intakeCoral).until(this::getCoral).andThen(this::stopCoral);
  }

  public Command outakeCoralCommand() {
    return new InstantCommand(this::outakeCoral).until(()->!getCoral()).andThen(new WaitCommand(0.2)).andThen(this::stopCoral);
  }

  public Command intakeAlgeaCommand() {
    return new InstantCommand(this::intakeAlgae).until(this::getAlgae).andThen(this::holdAlgea);
  }

  public Command outakeAlgeaCommand() {
    return new InstantCommand(this::outakeAlgae).until(()->!getAlgae()).andThen(new WaitCommand(0.2)).andThen(this::stopAlgea);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("EndEffector/hasCoral", getCoral());
    SmartDashboard.putBoolean("EndEffector/hasAlgae", getAlgae());
  }
}
