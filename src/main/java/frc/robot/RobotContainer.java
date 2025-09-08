// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.SwerveChassisConstants;
import frc.robot.commands.DriveSwerve;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.SwerveChassis;
import frc.robot.subsystems.SwerveModule;
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

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final CustomController DRIVER = new CustomController(Constants.OperatorConstants.kDriverControllerPort, CustomControllerType.XBOX, Constants.OperatorConstants.kDeadband, 1);

  private final SwerveChassis chassis;
  private final BlueShiftOdometry m_odometry;
  private final LimelightOdometryCamera m_limelight3G;
  private final SendableChooser<Command> m_autonomousChooser;

  private final Elevator elevator;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    this.chassis = new SwerveChassis(
      new SwerveModule(Constants.SwerveModuleConstants.kFrontLeftOptions),
      new SwerveModule(Constants.SwerveModuleConstants.kFrontRightOptions),
      new SwerveModule(Constants.SwerveModuleConstants.kBackLeftOptions),
      new SwerveModule(Constants.SwerveModuleConstants.kBackRightOptions),
      new Gyro(new GyroIOPigeon(Constants.SwerveChassisConstants.kGyroDevice))
    );

    this.elevator = new Elevator();

    this.m_limelight3G = new LimelightOdometryCamera("limelight_threeg", false, true, VisionOdometryFilters::visionFilter);

    this.m_odometry = new BlueShiftOdometry(
      Constants.SwerveChassisConstants.PhysicalModel.kDriveKinematics, 
      chassis::getHeading,
      chassis::getModulePositions,
      new Pose2d(),
      0.02,
      m_limelight3G
    );

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

    SmartDashboard.putData("Chassis/ResetTurningEncoders", new InstantCommand(chassis::resetTurningEncoders).ignoringDisable(true));

    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {
    DRIVER.topButton().whileTrue(elevator.setPostitionCommand(2.4));
    DRIVER.leftButton().whileTrue(elevator.setPostitionCommand(1.5));
    DRIVER.bottomButton().whileTrue(elevator.setPostitionCommand(0.5));
<
    this.chassis.setDefaultCommand(new DriveSwerve(
        chassis,
        () -> -DRIVER.getLeftY(),
        () -> -DRIVER.getLeftX(),
        () -> DRIVER.getLeftTrigger() - DRIVER.getRightTrigger(),
        () -> !DRIVER.bottomButton().getAsBoolean()
      )
    );

    this.elevator.setDefaultCommand(elevator.setPostitionCommand(0));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }
}
