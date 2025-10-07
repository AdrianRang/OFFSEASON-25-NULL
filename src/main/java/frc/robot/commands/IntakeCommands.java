package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.RobotState;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.Intakexer;
import frc.robot.subsystems.Arm.ArmPosition;
import frc.robot.subsystems.Elevator.ElevatorPosition;

public class IntakeCommands {
    public static Command completeIntakeCommand(Intakexer intake, Arm arm, Elevator elevator, EndEffector endEffector) {
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                intake.intakeWaitCommand(),
                elevator.setPostitionWaitCommand(ElevatorPosition.INTAKE)
                    .andThen(arm.setPostionWaitCommand(ArmPosition.INTAKE))
            ),
            passCommand(intake, endEffector),
            // new WaitUntilCommand(endEffector::hasCoral), //! END EFECTOR DOES NOT YET CHECK FOR CORAL
            new WaitCommand(1),
            ScoringCommands.setRobotState(RobotState.HOME, arm, elevator),
            intake.stopCommand(),
            endEffector.stopCommand()
        );
    }

    public static Command completeIntakeCommand_R(Intakexer intake, Arm arm, Elevator elevator, EndEffector endEffector) {
        return new SequentialCommandGroup(
            new SequentialCommandGroup(
                elevator.setPostitionWaitCommand(ElevatorPosition.INTAKE),
                arm.setPostionWaitCommand(ArmPosition.INTAKE)
                ),
                
                new InstantCommand(endEffector::intakeCoral, endEffector),
                intake.intakeCommand(),
            // STARTS ALL INTAKE COMPONENTS

            new WaitUntilCommand(() -> intake.hasCoral()),
            new WaitUntilCommand(() -> !intake.hasCoral()),

            ScoringCommands.setRobotState(RobotState.HOME, arm, elevator),
            intake.stopCommand(),
            endEffector.stopCommand()
        );
    }
    
    /**
     * Make sure you only use this while the robot is in intake position
    */
    public static Command passCommand(Intakexer intake, EndEffector endEffector) {
        // ! Ends until the end effector has the coral
        // TODO: Lengthen timeout after limit switch is installed on the end effector
        return intake.passCommand()
            .alongWith(endEffector.intakeCoralCommand())
            .withTimeout(0.5);
    }
}
