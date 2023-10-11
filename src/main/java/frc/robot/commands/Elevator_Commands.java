package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Subsystems.Elevator;

public class Elevator_Commands {
    private final Elevator elevator;
    private static Timer timer;

    public Elevator_Commands(Elevator elevator) {
        this.elevator = elevator;
        timer = new Timer();
    }

   public Command setPosition(double pos) {
        return new InstantCommand(() -> elevator.setPos(pos));
   }

   public Command setSpeed(double speed) {
        return new InstantCommand(() -> elevator.set(speed));
   }

   public Command autoSetSpeed(double speed, double time) {
        return new FunctionalCommand(
            () -> timer.restart(),
            () -> elevator.set(speed),
            (Boolean finished) -> {
                elevator.set(0.0);
            },
            () -> {
                if (timer.hasElapsed(time)) {
                    return true;
                } else {
                    return false;
                }
            },
            elevator);
    }
}