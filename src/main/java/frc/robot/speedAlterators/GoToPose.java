package frc.robot.speedAlterators;

import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import lib.BlueShift.control.SpeedAlterator;
import frc.robot.Constants.SwerveChassisConstants;

public class GoToPose extends SpeedAlterator{
    private final Supplier<Pose2d> poseSupplier;
    private final Pose2d targetPose;

    public GoToPose(Supplier<Pose2d> poseSupllier, Pose2d targetPose) {
        this.poseSupplier = poseSupllier;
        this.targetPose = targetPose;
    }

    @Override
    public void onEnable() {
        Pose2d pose = poseSupplier.get();
        SwerveChassisConstants.PoseControllers.translationXPID.reset(pose.getX());
        SwerveChassisConstants.PoseControllers.translationYPID.reset(pose.getY());
        SwerveChassisConstants.PoseControllers.rotationPID.reset(pose.getRotation().getRotations());
    }

    @Override
    public void onDisable() {}

    public ChassisSpeeds alterSpeed(ChassisSpeeds speeds, boolean robotRelative) {
        Pose2d pose = poseSupplier.get();

        double xSpeed = SwerveChassisConstants.PoseControllers.translationXPID.calculate(pose.getX(), targetPose.getX());
        double ySpeed = SwerveChassisConstants.PoseControllers.translationYPID.calculate(pose.getY(), targetPose.getY());
        double rotSpeed = SwerveChassisConstants.PoseControllers.rotationPID.calculate(pose.getRotation().getRotations(), targetPose.getRotation().getRotations());
       
        SmartDashboard.putNumber("Alterators/pose/rot", rotSpeed);
        SmartDashboard.putNumber("Alterators/pose/x", xSpeed);
        SmartDashboard.putNumber("Alterators/pose/y", ySpeed);
        Logger.recordOutput("Alterators/pose/desiredPose", targetPose);

        return ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotSpeed, pose.getRotation());
    }
}
