// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorPosition;
import frc.robot.subsystems.Arm.ArmPosition;
import lib.BlueShift.constants.CTRECANDevice;
import lib.BlueShift.constants.PIDFConstants;
import lib.BlueShift.constants.SwerveModuleOptions;
import lib.BlueShift.utils.SwerveChassis;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.path.PathConstraints;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes) 
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final double kDeadband = 0.05;
  }

  public static class SwerveChassisConstants {
    public static final CTRECANDevice kGyroDevice = new CTRECANDevice(34, "*");

    public static final class PhysicalModel {
      // * MAX DISPLACEMENT SPEED (and acceleration)
      public static final LinearVelocity kMaxSpeed = MetersPerSecond.of(4.0);
      public static final LinearAcceleration kMaxAcceleration = MetersPerSecond.per(Second).of(5.0);
      public static final LinearAcceleration kMaxDeceleration = MetersPerSecond.per(Second).of(-5.0);

      // * MAX ROTATIONAL SPEED (and acceleration)
      public static final AngularVelocity kMaxAngularSpeed = DegreesPerSecond.of(270.0);
      public static final AngularAcceleration kMaxAngularAcceleration = DegreesPerSecond.per(Second).of(360.0);
      public static final AngularAcceleration kMaxAngularDeceleration = DegreesPerSecond.per(Second).of(-360.0);

      // * Slew rate limiters
      public static final SlewRateLimiter yLimiter = new SlewRateLimiter(
          Constants.SwerveChassisConstants.PhysicalModel.kMaxAcceleration.in(MetersPerSecondPerSecond),
          Constants.SwerveChassisConstants.PhysicalModel.kMaxDeceleration.in(MetersPerSecondPerSecond), 0);
      public static final SlewRateLimiter xLimiter = new SlewRateLimiter(
          Constants.SwerveChassisConstants.PhysicalModel.kMaxAcceleration.in(MetersPerSecondPerSecond),
          Constants.SwerveChassisConstants.PhysicalModel.kMaxDeceleration.in(MetersPerSecondPerSecond), 0);
      public static final SlewRateLimiter rotLimiter = new SlewRateLimiter(
          Constants.SwerveChassisConstants.PhysicalModel.kMaxAngularAcceleration.in(RadiansPerSecond.per(Second)),
          Constants.SwerveChassisConstants.PhysicalModel.kMaxAngularDeceleration.in(RadiansPerSecond.per(Second)), 0);

      // Drive wheel diameter
      public static final Distance kWheelDiameter = Inches.of(4);

      // Gear ratios
      public static final double kDriveMotorGearRatio = 1.0 / 6.75; // 6.12:1 Drive
      public static final double kTurningMotorGearRatio = 1.0 / (150 / 7); // 12.8:1 Steering

      // Conversion factors (Drive Motor)
      public static final double kDriveEncoder_RotationToMeter = kDriveMotorGearRatio * kWheelDiameter.in(Meters);
      public static final double kDriveEncoder_RPMToMeterPerSecond = kDriveEncoder_RotationToMeter / 60.0;

      // Conversion factors (Turning Motor)
      public static final double kTurningEncoder_Rotation = kTurningMotorGearRatio;
      public static final double kTurningEncoder_RPS = kTurningEncoder_Rotation / 60.0;

      // Robot Without bumpers measures
      public static final Distance kTrackWidth = Inches.of(26);
      public static final Distance kWheelBase = Inches.of(26);

      // Create a kinematics instance with the positions of the swerve modules
      public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
          SwerveChassis.sizeToModulePositions(kTrackWidth.in(Meters), kWheelBase.in(Meters)));

      // Path constraints
      public static final PathConstraints kPathConstraints = new PathConstraints(kMaxSpeed, kMaxAcceleration,
          kMaxAngularSpeed, kMaxAngularAcceleration);
    }

    public static final class AutonomousConstants {
      public static final PIDConstants kTranslatePIDConstants = new PIDConstants(4.0, 0.0, 0.0);
      public static final PIDConstants kRotatePIDConstants = new PIDConstants(3.5, 0.0, 0.0);
    }
  }

  // * Swerve modules configuration
  public static final class SwerveModuleConstants {
    // * Motor config
    // Ramp rates
    public static final double kDriveMotorRampRate = 0;
    public static final double kTurningMotorRampRate = 0;

    // Current limits
    public static final int kDriveMotorCurrentLimit = 40;
    public static final int kDriveMotorLowerCurrentLimit = 30;
    public static final int kTurningMotorCurrentLimit = 30;

    // * PID
    public static final PIDFConstants kTurningPIDConstants = new PIDFConstants(1.57);

    // * Swerve modules options
    public static final SwerveModuleOptions kFrontLeftOptions = new SwerveModuleOptions()
        .setDriveMotorID(2)
        .setTurningMotorID(3)
        .setAbsoluteEncoderCANDevice(new CTRECANDevice(4, "*"))
        .setName("Front Left");

    public static final SwerveModuleOptions kFrontRightOptions = new SwerveModuleOptions()
        .setDriveMotorID(5)
        .setTurningMotorID(6)
        .setAbsoluteEncoderCANDevice(new CTRECANDevice(7, "*"))
        .setName("Front Right");

    public static final SwerveModuleOptions kBackLeftOptions = new SwerveModuleOptions()
        .setDriveMotorID(8)
        .setTurningMotorID(9)
        .setAbsoluteEncoderCANDevice(new CTRECANDevice(10, "*"))
        .setName("Back Left");

    public static final SwerveModuleOptions kBackRightOptions = new SwerveModuleOptions()
        .setDriveMotorID(11)
        .setTurningMotorID(12)
        .setAbsoluteEncoderCANDevice(new CTRECANDevice(13, "*"))
        .setName("Back Right");
  }

  public static final class LEDConstants {
    public static final CTRECANDevice kCandleId = new CTRECANDevice(41, "*");
    public static final double kBrightness = 0.5;
    public static final int kSideLedNum = 21;
    public static final int kTotalLedNum = kSideLedNum;
    public static final LEDStripType kType = LEDStripType.GRB;

    public static final class LEDAnimations {
        public static final RainbowAnimation kThinkingAnimation = new RainbowAnimation(LEDConstants.kBrightness, 0.85, kTotalLedNum);
        public static final SingleFadeAnimation kIdleAnimation = new SingleFadeAnimation(0, 0, 255, 0, 0.5, 20);
        public static final SingleFadeAnimation kTeleopAnimation = new SingleFadeAnimation(0, 0, 255, 0, 1, 20);
        public static final LarsonAnimation kIntakeStartAnimation = new LarsonAnimation(0, 0, 255, 0, 0.75, kTotalLedNum, BounceMode.Front, 1);
        public static final StrobeAnimation kIntakeCompleteAnimation = new StrobeAnimation(0, 0, 255, 0, 0.5, kTotalLedNum);
        public static final LarsonAnimation kAutoAnimation = new LarsonAnimation(0, 0, 255, 0, 0.85, kTotalLedNum, BounceMode.Front, 3);
    }
  }

  public static final class ElevatorConstants {
    public static final int leftMotorId = 57;
    public static final int rightMotorId = 58;
    public static final double kMotorRampRate = 0.0;
    public static final int kMototCurrentLimit = 40;

    //TODO: Update values
    public static final double kMaxHeight = 2.5; // Meters
    public static final double kMinHeight = 0; // Meters
    //! Encoders cannot have negative conversion factors, the motor has to be inverted so the value is inverted
    public static final double kRotationToHeightRatio = kMaxHeight / 35.0; // Rotations to meters
    public static final double kPositionEpsilon = 0.2;


    public static final double kBumpZone = 0.2;

    // PID controller
    public static final ProfiledPIDController pidController = new ProfiledPIDController(
      3.0,
      0.0,
      0.0,
      new TrapezoidProfile.Constraints(130, 50)
    );

    // Feedforward
    // TODO: CAREFULY CHECK THE BEHAVIOR OF THE ELEVATOR WITH FF
    // TODO: Tune kS manually, to tune it, set all other values to 0, then slowly increase the value until the elevator moves ever so slightly
    // Calculated with: https://www.reca.lc/linear?angle=%7B%22s%22%3A90%2C%22u%22%3A%22deg%22%7D&currentLimit=%7B%22s%22%3A40%2C%22u%22%3A%22A%22%7D&efficiency=90&limitAcceleration=0&limitDeceleration=0&limitVelocity=0&limitedAcceleration=%7B%22s%22%3A400%2C%22u%22%3A%22in%2Fs2%22%7D&limitedDeceleration=%7B%22s%22%3A50%2C%22u%22%3A%22in%2Fs2%22%7D&limitedVelocity=%7B%22s%22%3A10%2C%22u%22%3A%22in%2Fs%22%7D&load=%7B%22s%22%3A20%2C%22u%22%3A%22lbs%22%7D&motor=%7B%22quantity%22%3A2%2C%22name%22%3A%22NEO%20Vortex%2A%22%7D&ratio=%7B%22magnitude%22%3A20%2C%22ratioType%22%3A%22Reduction%22%7D&spoolDiameter=%7B%22s%22%3A1%2C%22u%22%3A%22in%22%7D&travelDistance=%7B%22s%22%3A2.5%2C%22u%22%3A%22m%22%7D
    public static final ElevatorFeedforward feedforward = new ElevatorFeedforward(0.0, 0.08, 26.60);
  }

  public static final class ArmConstants {
    // CAN IDs
    public static final int kMotorId = 17;
    public static final int kAbsoluteEncoderId = 45;
    
    // Motor config
    public static final double kMotorRampRate = 0;
    public static final int kMotorCurrentLimit = 40;
    public static final double kConversionFactor = 1.0 / 20.0 / 4.0; // Gearbox reduction + sprocket ratio (20:1 * 4:1)

    // Angle limits
    public static final Angle kMin = Degrees.of(15);
    public static final Angle kMax = Degrees.of(250);

    // PID controller
    public static final ProfiledPIDController pidController = new ProfiledPIDController(
      25.0,
      0.0,
      0.0,
      new TrapezoidProfile.Constraints(3.0, 2.0)
    );

    // Feedforward
    // Calculated with: https://www.reca.lc/arm?armMass=%7B%22s%22%3A15%2C%22u%22%3A%22lbs%22%7D&comLength=%7B%22s%22%3A17%2C%22u%22%3A%22in%22%7D&currentLimit=%7B%22s%22%3A40%2C%22u%22%3A%22A%22%7D&efficiency=90&endAngle=%7B%22s%22%3A180%2C%22u%22%3A%22deg%22%7D&iterationLimit=10000&motor=%7B%22quantity%22%3A1%2C%22name%22%3A%22NEO%20Vortex%2A%22%7D&ratio=%7B%22magnitude%22%3A80%2C%22ratioType%22%3A%22Reduction%22%7D&startAngle=%7B%22s%22%3A0%2C%22u%22%3A%22deg%22%7D
    // TODO: CAREFULY CHECK THE BEHAVIOR OF THE ARM WITH FF
    // ! KV seems high, check it
    // ? No kS gain?
    public static final ArmFeedforward feedforward = new ArmFeedforward(0.0, 0.04, 0.075);
  }

  public static final class EndEffectorConstants {
    public static final int kCoralMotorID = 50;
    public static final int kAlgaeMotorID = 51;
    public static final int kCoralSwitchID = 0;

    public static final class CoralConstants {
      public static final double intakeSpeed = -0.8;
      public static final double outakeSpeed = 0.5;

      public static final int currentLimit = 40;
    }

    public static final class AlgeaConstants {
      public static final double intakeCurrent = -3;
      public static final double outakeCurrent = 10;

      public static final double checkCurrent = 30;

      public static final int currentLimit = 40;

      // TODO
      public static final int kP = 0;
      public static final int kI = 0;
      public static final int kD = 0;
    }
  }

  public static enum RobotState {
    HOME(ArmPosition.IDLE, ElevatorPosition.HOME),
    L1(ArmPosition.PLACE_L1, Elevator.ElevatorPosition.L1),
    L2(ArmPosition.PLACE_L234, Elevator.ElevatorPosition.L2),
    L3(ArmPosition.PLACE_L234, Elevator.ElevatorPosition.L3);
    
    private ArmPosition armPosition;
    private ElevatorPosition elevatorPosition;

    private RobotState(ArmPosition armPosition, ElevatorPosition elevatorPosition) {
      this.armPosition = armPosition;
      this.elevatorPosition = elevatorPosition;
    }

    public ArmPosition getArmPosition() {return armPosition;}
    public ElevatorPosition getElevatorPosition() {return elevatorPosition;}
  }

  public static final double startupStatusSignalTimeout = 20;
  public static final double deviceCheckPeriod = 5;
}
