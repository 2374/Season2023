package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ManipulatorSubsystem extends SubsystemBase {
    private final WPI_TalonFX motor;

    public ManipulatorSubsystem() {
        motor = new WPI_TalonFX(Constants.MANIPULATOR_MOTOR_PORT, Constants.CANIVORE_CAN_BUS_NAME);
        motor.setNeutralMode(NeutralMode.Brake);
    }

    public void intake() {
        motor.set(.5);
    }

    public void outtake() {
        motor.set(-.5);
    }

    public void stop() {
        motor.stopMotor();
    }
}
