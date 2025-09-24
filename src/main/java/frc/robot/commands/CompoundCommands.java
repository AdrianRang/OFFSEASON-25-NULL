package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.RobotState;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

public class CompoundCommands {
    static Command setRobotState(RobotState state, Arm arm, Elevator elevator) {
        return new SequentialCommandGroup();
    }
}
