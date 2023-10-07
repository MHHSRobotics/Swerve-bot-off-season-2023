package frc.robot.Subsystems;

import frc.robot.Constants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class Elevator extends SubsystemBase {
    private Joystick controller;

    private static WPI_TalonSRX motor1;
    private static WPI_TalonSRX motor2;

    private static DigitalInput lowerLimitSwitch;
    private static DigitalInput upperLimitSwitch;

    private static Encoder encoder;
    private static PIDController PID;
    private static TrapezoidProfile TP;
    private static TrapezoidProfile.Constraints TP_Constraints;

    private static double goal;
    private static double position;
    private static double kV;
    private static double t;
    private static boolean profilingActive;

    public Elevator(Joystick controller) {
        this.controller = controller;

        motor1 = new WPI_TalonSRX(Constants.ElevatorConstants.motor1Port);
        motor2 = new WPI_TalonSRX(Constants.ElevatorConstants.motor2Port);

        lowerLimitSwitch = new DigitalInput(Constants.ElevatorConstants.lowerLimitPort);
        upperLimitSwitch = new DigitalInput(Constants.ElevatorConstants.upperLimitPort);

        motor1.configFactoryDefault();
        motor2.configFactoryDefault();

        motor1.setNeutralMode(NeutralMode.Brake);
        motor2.setNeutralMode(NeutralMode.Brake);

        //encoder = new Encoder(0, 1);
        t = 0.0;
        profilingActive = false;

        PID = new PIDController(Constants.ElevatorConstants.kP, Constants.ElevatorConstants.kI, Constants.ElevatorConstants.kD);
        TP_Constraints = new TrapezoidProfile.Constraints(Constants.ElevatorConstants.maxVelo, Constants.ElevatorConstants.maxAccel);
    }

    public void periodic() {
        t += 0.02;
        //position = encoder.getDistance();
        position += kV*0.02; //Simulated encoder
        if (Math.abs(controller.getRawAxis(1)) > 0.1) {
            kV = -controller.getRawAxis(1);
            goal = position;
            if (checkLimitSwitches()) {
                motor1.set(kV);
                motor2.set(kV);
            } else {
                motor1.set(0.0);
                motor2.set(0.0);
                kV = 0.0;
            }
            profilingActive = false;
        } else {
            if (profilingActive) {
                var target = TP.calculate(t);
                kV = PID.calculate(position, target.position);
            } else {
                kV = PID.calculate(position, goal);
            }
            if (kV < 0.01) {
                kV = 0;
            }
            
            if (checkLimitSwitches()) {
                motor1.setVoltage(kV);
                motor2.setVoltage(kV);
            } else {
                motor1.set(0.0);
                motor2.set(0.0);
                kV = 0.0;
            }
        }
        System.out.println("Velocity: "+kV+" Position: "+position+" Goal: "+goal);
    }

    private boolean checkLimitSwitches() {
        if (controller.getRawButton(5) && kV > 0.0) {
            return false;
        } else if (controller.getRawButton(6) && kV < 0.0) {
            return false;
        } else {
            return true;
        }
    }

    public void set(double x) {
        goal = x;
        TP = new TrapezoidProfile(TP_Constraints, new TrapezoidProfile.State(goal, 0.0), new TrapezoidProfile.State(position, kV));
        t = 0.0;
        profilingActive = true;
    }

    public double get() {
        return position;
    }
}
