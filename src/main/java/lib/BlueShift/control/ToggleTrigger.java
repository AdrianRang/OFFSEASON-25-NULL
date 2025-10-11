package lib.BlueShift.control;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ToggleTrigger extends Trigger {
  private boolean state = false;

  public ToggleTrigger(Trigger baseTrigger) {
    super(() -> baseTrigger.getAsBoolean());
    baseTrigger.onTrue(new InstantCommand(() -> {state = !state; Logger.recordOutput("AlgaeMode", state);}));
  }

  @Override
  public boolean getAsBoolean() {
    return state;
  }
}