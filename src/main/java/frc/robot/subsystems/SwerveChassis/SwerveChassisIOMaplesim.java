package frc.robot.subsystems.SwerveChassis;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.function.Consumer;

import org.ironmaple.simulation.drivesims.GyroSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SwerveChassisConstants;
import frc.robot.Constants.SwerveModuleConstants;
import lib.BlueShift.control.SpeedAlterator;

public class SwerveChassisIOMaplesim implements SwerveChassisIO {
    // * Gyro sim
    GyroSimulation gyro;

    // * Swerve modules sim
    SwerveModuleMaplesim frontLeft;
    SwerveModuleMaplesim frontRight;
    SwerveModuleMaplesim backLeft;
    SwerveModuleMaplesim backRight;

    // * State
    private boolean drivingRobotRelative = false;
    private ChassisSpeeds speeds = new ChassisSpeeds();
    private SpeedAlterator speedAlterator = null;

    public SwerveChassisIOMaplesim(GyroSimulation gyro, SwerveModuleSimulation frontLeft, SwerveModuleSimulation frontRight, SwerveModuleSimulation backLeft, SwerveModuleSimulation backRight) {
        this.gyro = gyro;
        this.frontLeft = new SwerveModuleMaplesim(frontLeft, "FrontLeft");
        this.frontRight = new SwerveModuleMaplesim(frontRight, "FrontRight");
        this.backLeft = new SwerveModuleMaplesim(backLeft, "BackLeft");
        this.backRight = new SwerveModuleMaplesim(backRight, "BackRight");
    }

    public Rotation2d getHeading() {
        return gyro.getGyroReading();
    }

    public void zeroHeading() {
        gyro.setRotation(Rotation2d.kZero);
    }

    public ChassisSpeeds getRobotRelativeChassisSpeeds() {
        if (this.drivingRobotRelative) return this.speeds;
        else return ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getHeading());
    }

    // TODO
    public SwerveModuleState[] getModuleTargetStates() {
        return new SwerveModuleState[] {
            frontLeft.getTargetState(),
            frontRight.getTargetState(),
            backLeft.getTargetState(),
            backRight.getTargetState(),
        };
    }

    // TODO
    public SwerveModuleState[] getModuleRealStates() {
        return new SwerveModuleState[] {
            frontLeft.getRealState(),
            frontRight.getRealState(),
            backLeft.getRealState(),
            backRight.getRealState(),
        };
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition(),
        };
    }

    /**
     * Set the module states
     * @param states
     */
    public void setModuleStates(SwerveModuleState[] states) {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.SwerveChassisConstants.PhysicalModel.kMaxSpeed.in(MetersPerSecond));
        frontLeft.setTargetState(states[0]);
        frontRight.setTargetState(states[1]);
        backLeft.setTargetState(states[2]);
        backRight.setTargetState(states[3]);
    }

    /**
     * Drive the robot with the provided speeds <b>(ROBOT RELATIVE)></b>
     * @param xSpeed
     * @param ySpeed
     * @param rotSpeed
     */
    public void drive(ChassisSpeeds speeds) {
        Logger.recordOutput("SwerveDrive/SpeedsAltered", speeds);
        if (speedAlterator != null) {
            this.speeds = speedAlterator.alterSpeed(speeds, drivingRobotRelative);
        } else {
            this.speeds = speeds;
        }
        Logger.recordOutput("SwerveDrive/SpeedsUnaltered", speeds);

        // Convert speeds to module states
        SwerveModuleState[] m_moduleStates = Constants.SwerveChassisConstants.PhysicalModel.kDriveKinematics.toSwerveModuleStates(this.speeds);

        // Set the target states for the modules
        this.setModuleStates(m_moduleStates);
    }

    public void enableSpeedAlterator(SpeedAlterator alterator) {
        if (this.speedAlterator != alterator) alterator.onEnable();
        if (this.speedAlterator != null) this.speedAlterator.onDisable();
        this.speedAlterator = alterator;
    }

    public void disableSpeedAlterator() {
        if(this.speedAlterator != null) this.speedAlterator.onDisable();
        this.speedAlterator = null;
    }

    /**
     * Drive the robot with the provided speeds <b>(ROBOT RELATIVE)</b>
     * @param xSpeed
     * @param ySpeed
     * @param rotSpeed
     */
    public void driveFieldRelative(double xSpeed, double ySpeed, double rotSpeed) {
        this.drivingRobotRelative = false;
        ChassisSpeeds Speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotSpeed, this.getHeading());
        this.drive(Speeds);
    }

    /**
     * Drive the robot with the provided speeds <b>(FIELD RELATIVE)</b>
     * @param speeds
     */
    public void driveFieldRelative(ChassisSpeeds speeds) {
        this.drivingRobotRelative = false;
        this.drive(speeds);
    }

    /**
     * Drive the robot with the provided speeds <b>(ROBOT RELATIVE)</b>
     * @param xSpeed
     * @param ySpeed
     * @param rotSpeed
     */
    public void driveRobotRelative(double xSpeed, double ySpeed, double rotSpeed) {
        this.drivingRobotRelative = true;
        this.drive(new ChassisSpeeds(xSpeed, ySpeed, rotSpeed));
    }

    /**
     * Drive the robot with the provided speeds <b>(ROBOT RELATIVE)</b>
     * @param speeds
     */
    public void driveRobotRelative(ChassisSpeeds speeds) {
        this.drivingRobotRelative = true;
        this.drive(speeds);
    }
    
    public void stop() {
        this.drive(new ChassisSpeeds());
    }

    public void resetDriveEncoders() {
        frontLeft.resetDriveEncoder();
        frontRight.resetDriveEncoder();
        backLeft.resetDriveEncoder();
        backRight.resetDriveEncoder();
    }

    public void resetTurningEncoders() {
        frontLeft.resetTurnEncoder();
        frontRight.resetTurnEncoder();
        backLeft.resetTurnEncoder();
        backRight.resetTurnEncoder();
    }

    public double getDip() {
        return 0.0;
    }

    public class SwerveModuleMaplesim extends SubsystemBase {
        private final SwerveModuleSimulation moduleSimulation;
        private final SimulatedMotorController.GenericMotorController driveMotor;
        private final SimulatedMotorController.GenericMotorController turnMotor;

        private SwerveModuleState targetState = new SwerveModuleState();
        private PIDController turnController = SwerveModuleConstants.kTurningPIDConstants.toPIDController();

        private Angle driveZeroUnGearedPosition = Rotations.of(0);
        private Rotation2d steerFacingZero = Rotation2d.kZero;

        private String name;

        public SwerveModuleMaplesim(SwerveModuleSimulation moduleSimulation, String name) {
            this.moduleSimulation = moduleSimulation;
            this.driveMotor = moduleSimulation.useGenericMotorControllerForDrive(); // TODO: configure
            this.turnMotor = moduleSimulation.useGenericControllerForSteer(); // TODO: configure
            this.name = name;
        }

        public SwerveModuleState getRealState() {
            return moduleSimulation.getCurrentState();
        }

        public SwerveModuleState getTargetState() {
            return targetState;
        }

        public void setTargetState(SwerveModuleState state) {
            this.targetState = state;
        }

        public SwerveModulePosition getPosition() {
            Angle[] drivePositions = moduleSimulation.getCachedDriveEncoderUnGearedPositions();
            Rotation2d[] steerFacings = moduleSimulation.getCachedSteerAbsolutePositions();
            return new SwerveModulePosition(
                drivePositions[drivePositions.length - 1].minus(driveZeroUnGearedPosition).in(Rotations) * 3 * Constants.SwerveChassisConstants.PhysicalModel.kDriveEncoder_RotationToMeter,
                steerFacings[steerFacings.length - 1].minus(steerFacingZero)
            );
            
        }

        public void resetDriveEncoder() {
            driveZeroUnGearedPosition = moduleSimulation.getDriveEncoderUnGearedPosition();
        }

        public void resetTurnEncoder() {
            steerFacingZero = moduleSimulation.getSteerAbsoluteFacing();
        }

        @Override
        public void periodic() {
            double driveVolts = targetState.speedMetersPerSecond / SwerveChassisConstants.PhysicalModel.kMaxSpeed.in(MetersPerSecond) * 12;
            double turnVolts = turnController.calculate(moduleSimulation.getSteerAbsoluteFacing().getRotations(), targetState.angle.getRotations()) * 12;
            driveMotor.requestVoltage(Volts.of(driveVolts));
            turnMotor.requestVoltage(Volts.of(turnVolts));

            Logger.recordOutput("SwerveDriveSim/" + name + "/TargetState", targetState);
            Logger.recordOutput("SwerveDriveSim/" + name + "/RealState", this.getRealState());

            Logger.recordOutput("SwerveDriveSim/" + name + "/DriveVoltage", driveVolts);
            Logger.recordOutput("SwerveDriveSim/" + name + "/TurnVoltage", turnVolts);

            Logger.recordOutput("SwerveDriveSim/" + name + "/Position", getPosition());
        }
    }
}