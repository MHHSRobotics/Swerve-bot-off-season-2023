package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Subsystems.Intake;

public class Intake_Commands {
    private final Intake intake;
    private static Timer timer;

    public Intake_Commands(Intake intake) {
        this.intake = intake;
        timer = new Timer();
    }

   public Command runIntake() {
        return new InstantCommand(() -> intake.intake(true));
   }

   public Command reverseIntake() {
        return new InstantCommand(() -> intake.intake(false));
   }

   public Command stopIntake() {
        return new InstantCommand(() -> intake.stop());
    }

    public Command autoIntake(boolean direction, double time) {
        return new FunctionalCommand(
            () -> timer.restart(),
            () -> intake.intake(direction),
            (Boolean finished) -> {
                intake.stop();
            },
            () -> {
                if (timer.hasElapsed(time)) {
                    return true;
                } else {
                    return false;
                }
            },
        intake);
    }
}