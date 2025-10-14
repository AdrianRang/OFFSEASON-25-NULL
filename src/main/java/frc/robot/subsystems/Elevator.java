package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants.PhysicalModel;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static frc.robot.Constants.ElevatorConstants.*;

public class Elevator extends SubsystemBase {
  public static enum  ElevatorPosition {
    // TODO: RESET POSITIONS (Absolute encoder)
    // ? divide by motor gear ratio?
    ZERO(0.0),
    // - // ERROR SO I DONT FORGET TO CHANGE THESE
		HOME(0.33),
    
		L1(HOME.getPosition()),

		L2(0.53), 
		L2_ALGAE(L2.getPosition() + 1),

		L3(2.5),
		L3_ALGAE(L3.getPosition() + 1),

		L4(7.6),

		STATION(5.0 / 15.0),

    INTAKE(2.5 / 15.0),
    NET(60 / 15.0);

    
		private double position;
    
		private ElevatorPosition(double position) {
      // if(position > kMaxHeight || position < kMinHeight) return;
      this.position = position;
		}
    
		public double getPosition() {
      return position;
		}
	}
  
  // * (MASTER)
  private final SparkFlex leftMotor;
  private final SparkFlexConfig leftMotorConfig;
  
  private final RelativeEncoder leftEncoder;
  
  // * (SLAVE)
  private final SparkFlex rightMotor;
  private final SparkFlexConfig rightMotorConfig;
  
  private ElevatorPosition setpoint = ElevatorPosition.ZERO; 
  private boolean pidEnabled = false;

  private final CANcoder absoluteEncoder;

  // * SIMULATION
  private SparkFlexSim leftMotorSim;
  private SparkFlexSim rightMotorSim;

  private ElevatorSim elevatorSim;

  // * Visualization
  private final Mechanism2d mechanism;
  private final MechanismRoot2d mechRoot;
  private final MechanismLigament2d ligament;
  
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
    // nice
    
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

    this.absoluteEncoder = new CANcoder(kEncoderId);

    // * Create simulation (when simulating)
    if (RobotBase.isSimulation()) {
      this.leftMotorSim = new SparkFlexSim(leftMotor, DCMotor.getNeoVortex(1));
      this.rightMotorSim = new SparkFlexSim(rightMotor, DCMotor.getNeoVortex(1));

      this.elevatorSim = new ElevatorSim(
        DCMotor.getNeoVortex(2),
        PhysicalModel.gearing,
        PhysicalModel.carriageMass.in(Kilograms),
        PhysicalModel.spoolDiameter.in(Meters),
        kMinHeight.in(Meters),
        kMaxHeight.in(Meters),
        true,
        kMinHeight.in(Meters)
      );
    }

    // * Visualization
    this.mechanism = new Mechanism2d(3, kMaxHeight.in(Meters), new Color8Bit(255, 255, 255));
    this.mechRoot = mechanism.getRoot("ElevatorRoot", 1.5, kMinHeight.in(Meters));
    this.ligament = mechRoot.append(new MechanismLigament2d("ElevatorLigament", 0, 90, 20, new Color8Bit(0, 0, 255)));

    // Log position setpoints for debugging
    for(ElevatorPosition pos : ElevatorPosition.values()) {
      SmartDashboard.putData("Elevator/Setpoint/" + pos.name(), setPostitionCommand(pos).ignoringDisable(true));
    }

    // Log PID for debugging
    SmartDashboard.putData("Elevator/PID", pidController);

    SmartDashboard.putData("Elevator/SmartReset", new InstantCommand(this::smartResetEncoder));
    SmartDashboard.putData("Elevator/Reset", new InstantCommand(this::resetEncoder));
  }

  
  public void setEncoder(double position) {
    absoluteEncoder.setPosition(position);
  }

  public void resetEncoder() {
    setEncoder(0.0);
  }

  public void smartResetEncoder() {
    setEncoder(absoluteEncoder.getAbsolutePosition().getValueAsDouble());
  }

  public double getEncoderPosition() {
    // return leftEncoder.getPosition();
    return absoluteEncoder.getPosition().getValueAsDouble();
  }

  public ElevatorPosition getSetpoint() {
    return setpoint;
  }

  public void setVoltage(double voltage) {
    pidEnabled = false;
    leftMotor.setVoltage(voltage);
  }

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
  public void simulationPeriodic() {
    elevatorSim.setInput(leftMotor.getAppliedOutput() * RoboRioSim.getVInVoltage());
    elevatorSim.update(0.02);

    leftMotorSim.iterate(elevatorSim.getVelocityMetersPerSecond() / kRotationToHeightRatio, RoboRioSim.getVInVoltage(), 0.02);

    SmartDashboard.putNumber("Elevator/SimPosition", elevatorSim.getPositionMeters());
    ligament.setLength(elevatorSim.getPositionMeters());
  }
 
  @Override
  public void periodic() {
    double encoderPosition = getEncoderPosition();
    double pidResult = pidController.calculate(encoderPosition, setpoint.getPosition());
    // ? setpoint should be passed instead of pid value to ff?
    double ffResult = feedforward.calculate(setpoint.getPosition());

    if (pidEnabled) leftMotor.setVoltage(pidResult);

    // Update visualization
    // ligament.setLength(encoderPosition * kRotationToHeightRatio);

    SmartDashboard.putData("Elevator/Mechanism", mechanism);

    // // TO DO: these 2 are the same
    SmartDashboard.putNumber("Elevator/RawPosition", leftEncoder.getPosition());
    SmartDashboard.putNumber("Elevator/Position", getEncoderPosition());

    SmartDashboard.putNumber("Elevator/AbsolutePosition", absoluteEncoder.getAbsolutePosition().getValueAsDouble());
    // SmartDashboard.putNumber("Elevator/PostitionTop", absoluteEncoder.getPosition().getValueAsDouble());

    SmartDashboard.putBoolean("Elevator/IsAtPosition", isAtPosition());
    SmartDashboard.putNumber("Elevator/setpoint", setpoint.getPosition());

    SmartDashboard.putNumber("Elevator/LAppliedOutput", leftMotor.getAppliedOutput());
    SmartDashboard.putNumber("Elevator/RAppliedOutput", rightMotor.getAppliedOutput());

    SmartDashboard.putNumber("Elevator/PIDResult", pidResult);
    SmartDashboard.putNumber("Elevator/FFResult", ffResult);
  }
}
