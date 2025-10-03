package frc.robot.speedAlterators;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.SwerveChassisConstants;
import lib.BlueShift.control.SpeedAlterator;

public class LookController extends SpeedAlterator {
    private final Supplier<Double> x;
    private final Supplier<Double> y;
    private final Supplier<Rotation2d> angleSupplier;
    private final double deadzone;

    public LookController(Supplier<Rotation2d> angleSupplier, Supplier<Double> controllerX, Supplier<Double> controllerY, double deadzone) {
        this.angleSupplier = angleSupplier;
        this.x = controllerX;
        this.y = controllerY;
        this.deadzone = deadzone;
    }

    @Override
    public void onEnable() {

    }

    @Override
    public void onDisable() {
        
    }

    public ChassisSpeeds alterSpeed(ChassisSpeeds speeds, boolean robotRelative) {
        if(Math.abs(x.get()) < deadzone && Math.abs(y.get()) < deadzone) return speeds;

        double angle = -(Math.atan2(x.get(), y.get()) / (Math.PI*2)) % 1;

        double rotSpeed = -SwerveChassisConstants.PoseControllers.rotationPID.calculate(angleSupplier.get().getRotations() % 1, angle);
        SmartDashboard.putNumber("Alterators/LookJoystick/DesiredAngle", angle);
        SmartDashboard.putNumber("Alterators/LookJoystick/CurrentAngle", angleSupplier.get().getRotations() % 1);
        SmartDashboard.putNumber("Alterators/LookJoystick/Speed", rotSpeed);

        speeds.omegaRadiansPerSecond = rotSpeed;

        return speeds;
    }
}