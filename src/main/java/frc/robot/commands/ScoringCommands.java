package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.RobotState;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

public class ScoringCommands {
    private static RobotState currentState = RobotState.HOME;
    private static RobotState previousState = RobotState.HOME;

    public static Command setRobotState(RobotState next, Arm arm, Elevator elevator) {
        if (next == RobotState.INTAKE) {
            return new SequentialCommandGroup(
                elevator.setPostitionWaitCommand(next.getElevatorPosition()),
                arm.setPostionWaitCommand(next.getArmPosition())
            );
        } else {
            return new SequentialCommandGroup(
                arm.setPostionWaitCommand(next.getArmPosition()),
                elevator.setPostitionWaitCommand(next.getElevatorPosition())
            );
        }
    }

    static RobotState getCurrentState() {
        return currentState;
    }

    static RobotState getPreviousState() {
        return previousState;
    }
}
