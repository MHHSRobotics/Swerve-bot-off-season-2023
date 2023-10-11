package frc.robot.Commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Subsystems.Swerve;

public class Swerve_Commands {
    private final Swerve swerve;
    private static Timer timer;

    public Swerve_Commands(Swerve swerve) {
        this.swerve = swerve;
        timer = new Timer();
    }

    public Command move(double x, double y, double rotation) {
        return new InstantCommand(() -> swerve.drive(new Translation2d(x, y), rotation, true, false));
    }

    public Command autoMove(double x, double y, double rotation, double time) {
        return new FunctionalCommand(
            () -> timer.restart(),
            () -> swerve.drive(new Translation2d(x, y), rotation, true, false),
            (Boolean finished) -> {},
            () -> {
                if (timer.hasElapsed(time)) {
                    return true;
                } else {
                    return false;
                }
            },
            swerve);
    }

    public Command autoBalance() {
        return new FunctionalCommand(
            () -> timer.restart(),
            () -> swerve.balance(),
            (Boolean finished) -> {},
            () -> {
                if (swerve.isBalanced()) {
                    return true;
                } else {
                    return false;
                }
            },
            swerve);
    }
}
