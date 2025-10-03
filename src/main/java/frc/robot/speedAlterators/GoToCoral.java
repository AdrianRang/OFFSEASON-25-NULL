package frc.robot.speedAlterators;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.SwerveChassisConstants;
import frc.robot.LimelightHelpers.RawDetection;
import frc.robot.LimelightHelpers;
import lib.BlueShift.control.SpeedAlterator;

public class GoToCoral extends SpeedAlterator {
    private final int coralPipeline;
    private int lastPipeline = -1;
    private final String limelight;
    private final Supplier<Pose2d> poseSupplier;

    public GoToCoral(Supplier<Pose2d> poseSupplier, String limelightName, int coralPipeline) {
        this.poseSupplier = poseSupplier;
        this.limelight = limelightName;
        this.coralPipeline = coralPipeline;
    }

    @Override
    public void onEnable() {
        this.lastPipeline = (int)LimelightHelpers.getCurrentPipelineIndex(limelight);
        LimelightHelpers.setPipelineIndex(limelight, coralPipeline);

        Pose2d pose = poseSupplier.get();
        SwerveChassisConstants.PoseControllers.translationXPID.reset(pose.getX());
        SwerveChassisConstants.PoseControllers.translationYPID.reset(pose.getY());
        SwerveChassisConstants.PoseControllers.rotationPID.reset(pose.getRotation().getRotations());
    }

    @Override
    public void onDisable() {
        if (this.lastPipeline != -1) LimelightHelpers.setPipelineIndex(limelight, this.lastPipeline);
    }

    @Override
    public ChassisSpeeds alterSpeed(ChassisSpeeds speeds, boolean robotRelative) {
        RawDetection[] all = LimelightHelpers.getRawDetections(limelight);
        for (RawDetection det : all) {
            if (det.classId == 0) continue; // Ignore algae
            double tx = det.txnc;
	        double ty = det.tync;
            // TODO: Implement this (from citrus)
            // Translation2d coralTranslation = calcDistToCoral(tx, ty).minus(config.robotToCameraOffset.getTranslation().toTranslation2d()); // TODO: Subsistute with constants
            // Pose2d coralPose = poseSupplier.get().transformBy(new Transform2d(coralTranslation, new Rotation2d()));
        }
        return speeds;
    }
}