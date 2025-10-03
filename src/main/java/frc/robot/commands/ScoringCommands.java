package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.RobotState;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

public class ScoringCommands {
    private static RobotState currentState = RobotState.HOME;
    private static RobotState previousState = RobotState.HOME;

    public static Command setRobotState(RobotState state, Arm arm, Elevator elevator) {
        return new SequentialCommandGroup(
            arm.setPostionWaitCommand(state.getArmPosition()),
            elevator.setPostitionWaitCommand(state.getElevatorPosition())
        );
    }

    public static RobotState getCurrentState() {
        return currentState;
    }

    public static RobotState getPreviousState() {
        return previousState;
    }
}
