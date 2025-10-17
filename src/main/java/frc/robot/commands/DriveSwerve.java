package frc.robot.commands;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import frc.robot.Constants;
import frc.robot.Constants.SwerveChassisConstants;
import frc.robot.subsystems.SwerveChassis.SwerveChassis;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveSwerve extends Command {
  //* The swerve drive subsystem
  private final SwerveChassis chassis;

  //* The suppliers for the joystick values
  private final Supplier<Double> xSpeed;
  private final Supplier<Double> ySpeed;
  private final Supplier<Double> rotSpeed;
  private final Supplier<Boolean> fieldRelative;
  
  public double modifyAxis(double input, double scaleFactor, SlewRateLimiter slew) {
    // Get sign
    double sign = Math.signum(input);

    // Use absolute value for calculations (needed so slew rate limiter acts correctly against negative acceleration)
    input = Math.abs(input);
    
    // ! Deadband applied to joystick directly

    // Scale input
    input *= scaleFactor;

    // Apply rate limit
    slew.reset(input); // TODO: Check this
    input = slew.calculate(input);

    // Reapply the sign
    input *= sign;

    // Return the result
    return input;
  }

  public DriveSwerve(SwerveChassis chassis, Supplier<Double> x, Supplier<Double> y, Supplier<Double> rot, Supplier<Boolean> fieldRelative) {
    this.chassis = chassis;
    this.xSpeed = x;
    this.ySpeed = y;
    this.rotSpeed = rot;
    this.fieldRelative = fieldRelative;
    addRequirements(this.chassis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Get the joystick values
    double x = xSpeed.get();
    double y = ySpeed.get();
    double rot = rotSpeed.get();

    // Modify the axis
    x = modifyAxis(x, Constants.SwerveChassisConstants.PhysicalModel.kMaxSpeed.in(MetersPerSecond), SwerveChassisConstants.PhysicalModel.xLimiter);
    y = modifyAxis(y, Constants.SwerveChassisConstants.PhysicalModel.kMaxSpeed.in(MetersPerSecond), SwerveChassisConstants.PhysicalModel.yLimiter);
    rot = modifyAxis(rot, Constants.SwerveChassisConstants.PhysicalModel.kMaxAngularSpeed.in(RadiansPerSecond), SwerveChassisConstants.PhysicalModel.rotLimiter);
    
    // Drive the swerve drive
    if (this.fieldRelative.get()) {
      chassis.driveFieldRelative(x, y, rot);
    } else {
      chassis.driveRobotRelative(x, y, rot);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    chassis.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
