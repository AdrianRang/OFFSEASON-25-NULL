package frc.robot.subsystems.SwerveChassis;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import lib.BlueShift.control.SpeedAlterator;

// TODO: Use some default methods
public interface SwerveChassisIO {
    public Rotation2d getHeading();
    public void zeroHeading();

    public ChassisSpeeds getRobotRelativeChassisSpeeds();

    public SwerveModuleState[] getModuleTargetStates();
    public SwerveModuleState[] getModuleRealStates();

    public SwerveModulePosition[] getModulePositions();

    public void setModuleStates(SwerveModuleState[] states);

    public void drive(ChassisSpeeds speeds);

    public void enableSpeedAlterator(SpeedAlterator alterator);
    public void disableSpeedAlterator();

    public void driveFieldRelative(double vx, double vy, double omega);
    public void driveFieldRelative(ChassisSpeeds speeds);

    public void driveRobotRelative(double vx, double vy, double omega);
    public void driveRobotRelative(ChassisSpeeds speeds);

    public void stop();

    public default void xFormation() {};

    public void resetTurningEncoders();
    public void resetDriveEncoders();

    public double getDip();

    public default void resetEncoders() {
        resetTurningEncoders();
        resetDriveEncoders();
    }

    public default void periodic() {};
}
