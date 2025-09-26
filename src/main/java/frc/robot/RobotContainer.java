package frc.robot;

import frc.robot.Constants.SwerveChassisConstants;
import frc.robot.commands.DriveSwerve;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.SwerveChassis;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.Elevator.ElevatorPosition;
import frc.robot.subsystems.Gyro.Gyro;
import frc.robot.subsystems.Gyro.GyroIOPigeon;
import lib.Elastic;
import lib.Elastic.Notification;
import lib.Elastic.Notification.NotificationLevel;
import lib.BlueShift.control.CustomController;
import lib.BlueShift.control.CustomController.CustomControllerType;
import lib.BlueShift.odometry.swerve.BlueShiftOdometry;
import lib.BlueShift.odometry.vision.camera.LimelightOdometryCamera;
import lib.BlueShift.odometry.vision.camera.VisionOdometryFilters;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.DriveFeedforwards;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // * Controllers
  private final CustomController DRIVER = new CustomController(Constants.OperatorConstants.kDriverControllerPort, CustomControllerType.XBOX, Constants.OperatorConstants.kDeadband, 1);
  private final CustomController OPERATOR = new CustomController(Constants.OperatorConstants.kOperatorControllerPort, CustomControllerType.XBOX, Constants.OperatorConstants.kDeadband, 1);

  // * SwerveDrive
  private final SwerveChassis chassis;

  // * Odometry
  private final BlueShiftOdometry m_odometry;
  private final LimelightOdometryCamera m_limelight3G;

  // * Subsystems
  // Elevator
  private final Elevator elevator;
  
  // Arm
  private final Arm arm;

  // End effector
  private final EndEffector endEffector;

  // * Autonomous
  private final SendableChooser<Command> m_autonomousChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // * SwerveDrive
    this.chassis = new SwerveChassis(
      new SwerveModule(Constants.SwerveModuleConstants.kFrontLeftOptions),
      new SwerveModule(Constants.SwerveModuleConstants.kFrontRightOptions),
      new SwerveModule(Constants.SwerveModuleConstants.kBackLeftOptions),
      new SwerveModule(Constants.SwerveModuleConstants.kBackRightOptions),
      new Gyro(new GyroIOPigeon(Constants.SwerveChassisConstants.kGyroDevice))
    );

    // * Odometry
    // Cameras
    this.m_limelight3G = new LimelightOdometryCamera("limelight-threeg", false, true, VisionOdometryFilters::visionFilter);

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
    // Elevator
    this.elevator = new Elevator();

    // Arm
    // TODO: make it so it takes the encoder position
    this.arm = new Arm(() -> elevator.getSetpoint());

    // End effector
    this.endEffector = new EndEffector();

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

    // Auto chooser
    this.m_autonomousChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("AutoChooser", m_autonomousChooser);

    // Debug dashboard commands
    SmartDashboard.putData("Chassis/ResetTurningEncoders", new InstantCommand(chassis::resetTurningEncoders).ignoringDisable(true));
    SmartDashboard.putData("Elevator/ResetEncoder", new InstantCommand(elevator::resetEncoder).ignoringDisable(true));

    // Configure the trigger bindings
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
    
    // ? Changed these to onTrue because it's inefficient to have the driver holding the button
    DRIVER.povRight().onTrue(elevator.setPostitionCommand(ElevatorPosition.L3));
    DRIVER.povLeft().onTrue(elevator.setPostitionCommand(ElevatorPosition.L2));
    DRIVER.povDown().onTrue(elevator.setPostitionCommand(ElevatorPosition.L1));

    // ! OPERATOR
    // Algae
    // TODO: Try without limitswitch
    Trigger algaeMode = OPERATOR.leftTrigger();
    OPERATOR.leftButton().and(algaeMode).onTrue(endEffector.intakeAlgaeCommand());
    OPERATOR.topButton().and(algaeMode).onTrue(endEffector.outakeAlgaeCommand());

    // Coral
    OPERATOR.leftButton().onTrue(endEffector.intakeCoralCommand());
    OPERATOR.topButton().onTrue(endEffector.outakeCoralCommand());

    // ! TODO: If the command used to set the elevator position doesn't run constantly this will override it
    // this.elevator.setDefaultCommand(elevator.setPostitionCommand(ElevatorPosition.HOME));
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
