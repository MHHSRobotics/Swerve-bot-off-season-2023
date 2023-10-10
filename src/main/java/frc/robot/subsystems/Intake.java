package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private static CANSparkMax motor;
    private static SimDevice motorSim;
    private static SimDouble speedSim;

    public Intake() {
        motor = new CANSparkMax(Constants.IntakeConstants.intakePort, MotorType.kBrushless);
        motorSim = SimDevice.create("Intake Motor", 15);
        speedSim = motorSim.createDouble("Speed", Direction.kInput, 0.0);
    }

    public void intake(boolean dir) {
        if (dir) {
            motor.set(1.0);
            speedSim.set(1.0);
        } else {
            motor.set(-1.0);
            speedSim.set(-1.0);
        }
    }

    public void stop() {
        motor.set(0.0);
        speedSim.set(0.0);
    }

    public double get() {
        return motor.get();
    }
}
