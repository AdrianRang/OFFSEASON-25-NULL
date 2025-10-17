package frc.robot;

import frc.robot.Constants.RobotState;
import frc.robot.Constants.SwerveChassisConstants;
import frc.robot.commands.DriveSwerve;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.ScoringCommands;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.Intaxer;
import frc.robot.subsystems.IntaxerPivot;
import frc.robot.subsystems.SwerveChassis.SwerveChassis;
import frc.robot.subsystems.SwerveChassis.SwerveChassisIOMaplesim;
import frc.robot.subsystems.SwerveChassis.SwerveChassisIOReal;
import frc.robot.subsystems.SwerveChassis.SwerveChassisIOSim;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.Gyro.Gyro;
import frc.robot.subsystems.Gyro.GyroIOPigeon;
import lib.Elastic;
import lib.Elastic.Notification;
import lib.Elastic.Notification.NotificationLevel;
import lib.BlueShift.control.CustomController;
import lib.BlueShift.control.ToggleTrigger;
import lib.BlueShift.control.CustomController.CustomControllerType;
import lib.BlueShift.odometry.swerve.BlueShiftOdometry;
import lib.BlueShift.odometry.vision.camera.LimelightOdometryCamera;
import lib.BlueShift.odometry.vision.camera.PhotonOdometryCamera;
import lib.BlueShift.odometry.vision.camera.VisionOdometryFilters;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.DriveFeedforwards;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // * Controllers
  private final CustomController DRIVER = new CustomController(Constants.OperatorConstants.kDriverControllerPort, RobotBase.isReal() ? CustomControllerType.XBOX : CustomControllerType.PS5, Constants.OperatorConstants.kDeadband, 1);
  private final CustomController OPERATOR = new CustomController(Constants.OperatorConstants.kOperatorControllerPort, CustomControllerType.XBOX, Constants.OperatorConstants.kDeadband, 1);

  private SwerveDriveSimulation chassisSim;
  private final SwerveChassis chassis;

  private final BlueShiftOdometry m_odometry;
  private final LimelightOdometryCamera m_limelight3G;

  // * Subsystems
  private final Elevator elevator;
  private final Arm arm;
  private final EndEffector endEffector;
  private final Intaxer intake;

  private final IntaxerPivot pivot;

  // * Autonomous
  private final SendableChooser<Command> m_autonomousChooser;

  Trigger enableTrigger = new Trigger(DriverStation::isEnabled);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    if (RobotBase.isReal()) {
      this.chassis = new SwerveChassis(new SwerveChassisIOReal(
        new SwerveModule(Constants.SwerveModuleConstants.kFrontLeftOptions),
        new SwerveModule(Constants.SwerveModuleConstants.kFrontRightOptions),
        new SwerveModule(Constants.SwerveModuleConstants.kBackLeftOptions),
        new SwerveModule(Constants.SwerveModuleConstants.kBackRightOptions),
        new Gyro(new GyroIOPigeon(Constants.SwerveChassisConstants.kGyroDevice))
      ));
    } else {
      // this.chassisSim = new SwerveDriveSimulation(
      //   DriveTrainSimulationConfig.Default()
      //     .withCustomModuleTranslations(SwerveChassisConstants.PhysicalModel.kModuleTranslations)
      //     // .withRobotMass(null)
      //     // .withBumperSize(null, null)
      //     .withGyro(COTS.ofPigeon2())
      //     .withSwerveModule(new SwerveModuleSimulationConfig(
      //       DCMotor.getKrakenX60(1),
      //       DCMotor.getNEO(1),
      //       SwerveChassisConstants.PhysicalModel.kDriveMotorGearRatio,
      //       SwerveChassisConstants.PhysicalModel.kTurningMotorGearRatio,
      //       Volts.of(1),
      //       Volts.of(1),
      //       SwerveChassisConstants.PhysicalModel.kWheelDiameter,
      //       KilogramSquareMeters.of(0.02),
      //       1.0 // friction
      //     )),
      //     new Pose2d(3, 3, Rotation2d.kZero)
      // );
      // SimulatedArena.getInstance().addDriveTrainSimulation(chassisSim);

      // this.chassis = new SwerveChassis(new SwerveChassisIOMaplesim(
      //   chassisSim.getGyroSimulation(),
      //   chassisSim.getModules()[0],
      //   chassisSim.getModules()[1],
      //   chassisSim.getModules()[2],
      //   chassisSim.getModules()[3]
      // ));

      chassis = new SwerveChassis(new SwerveChassisIOSim());
    }

    // * Odometry
    // Cameras
    this.m_limelight3G = new LimelightOdometryCamera("limelight-threeg", false, true, VisionOdometryFilters::visionFilter);
    new PhotonOdometryCamera(null, new Transform3d(Meters.of(0.304), Meters.of(-0.288), Meters.of(-0.163), new Rotation3d(Degrees.of(0), Degrees.of(25), Degrees.of(90 - 75))), false, null);

    // Odometry
    this.m_odometry = new BlueShiftOdometry(
      Constants.SwerveChassisConstants.PhysicalModel.kDriveKinematics, 
      chassis::getHeading,
      chassis::getModulePositions,
      new Pose2d(),
      0.02,
      m_limelight3G
    );
    
    // * Subsystems
    this.elevator = new Elevator();

    this.arm = new Arm();

    this.endEffector = new EndEffector();

    this.intake = new Intaxer();

    this.pivot = new IntaxerPivot();

    // * Autonomous
    RobotConfig ppRobotConfig = null;
    try{
      ppRobotConfig = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      Elastic.sendNotification(new Notification(NotificationLevel.ERROR, "ERROR! COULD NOT LOAD PP ROBOT CONFIG", e.getMessage()));
      DriverStation.reportError("ERROR! COULD NOT LOAD PP ROBOT CONFIG", e.getStackTrace());
    }

    AutoBuilder.configure(
      m_odometry::getEstimatedPosition,
      m_odometry::resetPosition,
      chassis::getRobotRelativeChassisSpeeds,
      (ChassisSpeeds speeds, DriveFeedforwards ff) -> chassis.driveRobotRelative(speeds),
      new PPHolonomicDriveController(
        SwerveChassisConstants.AutonomousConstants.kTranslatePIDConstants,
        SwerveChassisConstants.AutonomousConstants.kRotatePIDConstants
      ),
      ppRobotConfig,
      () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
      chassis
    );

    this.m_autonomousChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("AutoChooser", m_autonomousChooser);

    // Debug dashboard commands
    SmartDashboard.putData("Chassis/ResetTurningEncoders", new InstantCommand(chassis::resetTurningEncoders).ignoringDisable(true));
    SmartDashboard.putData("Elevator/ResetEncoder", new InstantCommand(elevator::resetEncoder).ignoringDisable(true));
    // SmartDashboard.putData("Commands/completeIntake", IntakeCommands.completeIntakeCommand(intake, arm, elevator, endEffector));


    // Reset PIDs on enable
    enableTrigger.onTrue(new InstantCommand(() -> {
      elevator.resetPID();
    }));

    configureBindings();
  }

  private void configureBindings() {
    // ! DRIVER
    this.chassis.setDefaultCommand(new DriveSwerve(
        chassis,
        () -> -DRIVER.getLeftY(),
        () -> -DRIVER.getLeftX(),
        () -> DRIVER.getLeftTrigger() - DRIVER.getRightTrigger(),
        () -> !DRIVER.bottomButton().getAsBoolean()
      )
    );

    DRIVER.rightStickButton().onTrue(new InstantCommand(chassis::zeroHeading));
    
    // DRIVER.povRight().onTrue(elevator.setPostitionCommand(ElevatorPosition.L3));
    // DRIVER.povLeft().onTrue(elevator.setPostitionCommand(ElevatorPosition.L2));
    // DRIVER.povDown().onTrue(elevator.setPostitionCommand(ElevatorPosition.L1));

    // ! OPERATOR
    // Algae
    ToggleTrigger algaeMode = new ToggleTrigger(OPERATOR.leftBumper());

    OPERATOR.leftButton().and(() -> algaeMode.getAsBoolean()).onTrue(new RunCommand(endEffector::intakeAlgae, endEffector));
    OPERATOR.topButton().and(() -> algaeMode.getAsBoolean()).onTrue(new RunCommand(endEffector::outakeAlgae, endEffector));
    OPERATOR.topButton().and(() -> algaeMode.getAsBoolean()).onFalse(new RunCommand(endEffector::stopAlgae, endEffector));

    DRIVER.leftButton().and(() -> algaeMode.getAsBoolean()).onTrue(new RunCommand(endEffector::intakeAlgae, endEffector));
    DRIVER.topButton().and(() -> algaeMode.getAsBoolean()).onTrue(new RunCommand(endEffector::outakeAlgae, endEffector));
    DRIVER.topButton().and(() -> algaeMode.getAsBoolean()).onFalse(new RunCommand(endEffector::stopAlgae, endEffector));

    // Coral
    OPERATOR.leftButton().and(() -> !algaeMode.getAsBoolean()).onTrue(new RunCommand(endEffector::intakeCoral, endEffector));
    OPERATOR.leftButton().and(() -> !algaeMode.getAsBoolean()).onFalse(new RunCommand(endEffector::stopCoral, endEffector));

    OPERATOR.topButton().and(() -> !algaeMode.getAsBoolean()).onTrue(new RunCommand(endEffector::outakeCoral, endEffector));
    OPERATOR.topButton().and(() -> !algaeMode.getAsBoolean()).onFalse(new RunCommand(endEffector::stopCoral, endEffector));

    OPERATOR.rightButton().onTrue(IntakeCommands.completeIntakeCommand_R(intake, arm, elevator, endEffector));
    OPERATOR.bottomButton().onTrue(intake.ejectCommand());
    OPERATOR.bottomButton().onFalse(intake.stopCommand());

    DRIVER.topButton().and(() -> !algaeMode.getAsBoolean()).onTrue(new RunCommand(endEffector::outakeCoral, endEffector));
    DRIVER.topButton().and(() -> !algaeMode.getAsBoolean()).onFalse(new RunCommand(endEffector::stopCoral, endEffector));

    DRIVER.leftButton().and(() -> !algaeMode.getAsBoolean()).onTrue(new RunCommand(endEffector::intakeCoral, endEffector));
    DRIVER.leftButton().and(() -> !algaeMode.getAsBoolean()).onFalse(new RunCommand(endEffector::stopCoral, endEffector));

    DRIVER.rightButton().onTrue(IntakeCommands.completeIntakeCommand_R(intake, arm, elevator, endEffector));

    DRIVER.leftBumper().onTrue(elevator.setVoltageCommand(-2));
    DRIVER.leftBumper().onFalse(elevator.setVoltageCommand(0));
    DRIVER.rightBumper().onTrue(elevator.setVoltageCommand(6));
    DRIVER.rightBumper().onFalse(elevator.setVoltageCommand(0));

    OPERATOR.povUp().and(algaeMode.negate()).onTrue(ScoringCommands.setRobotState(RobotState.L4, arm, elevator));
    OPERATOR.povRight().and(algaeMode.negate()).onTrue(ScoringCommands.setRobotState(RobotState.L2, arm, elevator));
    OPERATOR.povLeft().and(algaeMode.negate()).onTrue(ScoringCommands.setRobotState(RobotState.L3, arm, elevator));
    
    OPERATOR.povUp().and(algaeMode).onTrue(ScoringCommands.setRobotState(RobotState.NET, arm, elevator));
    OPERATOR.povLeft().and(algaeMode).onTrue(ScoringCommands.setRobotState(RobotState.L3_ALGAE, arm, elevator));
    OPERATOR.povRight().and(algaeMode).onTrue(ScoringCommands.setRobotState(RobotState.L2_ALGAE, arm, elevator));
    
    OPERATOR.povDown().onTrue(ScoringCommands.setRobotState(RobotState.HOME, arm, elevator));

    DRIVER.povUp().and(algaeMode.negate()).onTrue(ScoringCommands.setRobotState(RobotState.L4, arm, elevator));
    DRIVER.povLeft().and(algaeMode.negate()).onTrue(ScoringCommands.setRobotState(RobotState.L3, arm, elevator));
    DRIVER.povRight().and(algaeMode.negate()).onTrue(ScoringCommands.setRobotState(RobotState.L2, arm, elevator));
    
    DRIVER.povUp().and(algaeMode).onTrue(ScoringCommands.setRobotState(RobotState.NET, arm, elevator));
    DRIVER.povLeft().and(algaeMode).onTrue(ScoringCommands.setRobotState(RobotState.L3_ALGAE, arm, elevator));
    DRIVER.povRight().and(algaeMode).onTrue(ScoringCommands.setRobotState(RobotState.L2_ALGAE, arm, elevator));
    
    DRIVER.povDown().onTrue(ScoringCommands.setRobotState(RobotState.HOME, arm, elevator));

    OPERATOR.backButton().onTrue(pivot.setDownCommand());
    OPERATOR.startButton().onTrue(pivot.setUpCommand());

    //! TEST
    // Auto lower elevator when dipping
    Trigger dipping = new Trigger(()->chassis.getDip()<45);
    Logger.recordOutput("dipping", dipping);
    // dipping.whileTrue(ScoringCommands.setRobotState(RobotState.HOME, arm, elevator));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autonomousChooser.getSelected();
  }
}