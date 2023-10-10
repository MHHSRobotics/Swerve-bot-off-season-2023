package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private static CANSparkMax motor;

    public Intake() {
        motor = new CANSparkMax(Constants.IntakeConstants.intakePort, MotorType.kBrushless);
    }

    public void intake(boolean dir) {
        if (dir) {
            motor.set(1.0);
        } else {
            motor.set(-1.0);
        }
    }

    public void stop() {
        motor.set(0.0);
    }

    public double get() {
        return motor.get();
    }
}
