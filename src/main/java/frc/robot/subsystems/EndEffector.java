// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import static frc.robot.Constants.EndEffectorConstants.*;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
  // private final SparkClosedLoopController algaePid;

  private final DigitalInput coralSwitch;

  private final Debouncer coralDebouncer = new Debouncer(0, DebounceType.kBoth);
  private final Debouncer algaeDebouncer = new Debouncer(0.1, DebounceType.kBoth);

  private boolean algae;

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
      // .closedLoop.pid(AlgeaConstants.kP, AlgeaConstants.kI, AlgeaConstants.kD);

    coralSwitch = new DigitalInput(kCoralSwitchID);

    coralMotor.configure(coralConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    algaeMotor.configure(algaeConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    // algaePid = algaeMotor.getClosedLoopController();
  }

  // TODO: Test Algea by current
  public void intakeAlgae() {
    algaeMotor.set(AlgeaConstants.intakeSpeed);
    // algaePid.setReference(AlgeaConstants.intakeCurrent, ControlType.kCurrent);
  }

  // public void holdAlgae() {
  //   algaeMotor.set(AlgeaConstants.holdSpeed);
  // }

  public void outakeAlgae() {
    algae = false;
    algaeMotor.set(AlgeaConstants.outakeSpeed);
    // algaePid.setReference(AlgeaConstants.outakeCurrent, ControlType.kCurrent);
  }

  public void stopAlgae() {
    algaeMotor.set(0);
    // algaePid.setReference(0, ControlType.kCurrent);
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
    return coralDebouncer.calculate(coralSwitch.get());
    // return false;
    // return coralDebouncer.calculate(coralMotor.getOutputCurrent() < CoralConstants.checkCurrent);
  }

  public boolean hasAlgae() {
    // return algaeMotor.getOutputCurrent() > AlgeaConstants.checkCurrent;
    // return false;
    return algae;
  }

  //? InstantCommand as the motor is set to a constant value no need to continuously run intakeCoral.
  public Command intakeCoralCommand() {
    return new RunCommand(this::intakeCoral)
      // .until(this::hasCoral)
      .andThen(new WaitCommand(2))
      .andThen(this::stopCoral);
  }

  public Command outakeCoralCommand() {
    return new InstantCommand(this::outakeCoral)
      .until(()->!hasCoral())
      .andThen(new WaitCommand(0.2))
      .andThen(this::stopCoral);
  }

  public Command intakeAlgaeCommand() {
    return new InstantCommand(this::intakeAlgae)
      .until(this::hasAlgae);
      // .andThen(this::holdAlgae);
  }

  public Command outakeAlgaeAutoCommand() {
    return new InstantCommand(this::outakeAlgae)
      .until(()->!hasAlgae())
      .andThen(new WaitCommand(0.2))
      .andThen(this::stopAlgae);
  }

  public Command outakeAlgaeCommand() {
    return new InstantCommand(this::outakeAlgae);
  }

  public Command stopCommand() {
    return new InstantCommand(()->{stopAlgae(); stopCoral();}, this);
  }

  @Override
  public void periodic() {
    if (algaeDebouncer.calculate(algaeMotor.getOutputCurrent() > AlgeaConstants.checkCurrent)) {
      algae = true;
    }

    if (hasAlgae()) {
      algaeMotor.set(AlgeaConstants.holdSpeed);
    }

    SmartDashboard.putBoolean("EndEffector/hasCoral", hasCoral());
    SmartDashboard.putBoolean("EndEffector/hasAlgae", hasAlgae());

    SmartDashboard.putNumber("EndEffector/CoralCurrent", coralMotor.getOutputCurrent());
    SmartDashboard.putNumber("EndEffector/AlgeaCurrent", algaeMotor.getOutputCurrent());
  }
}
