package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {

    private WPI_TalonFX leftShoulderMotor;
    private WPI_TalonFX rightShoulderMotorTwo;
    private MotorControllerGroup shoulderGroup;

    private WPI_TalonFX elbowMotor;

    private WPI_TalonFX wristMotor;

    // Encoders (IDK HOW YET)

    public ArmSubsystem() {
        leftShoulderMotor = new WPI_TalonFX(0, Constants.CANIVORE_CAN_BUS_NAME);
        rightShoulderMotorTwo = new WPI_TalonFX(0, Constants.CANIVORE_CAN_BUS_NAME);
        leftShoulderMotor.setInverted(true);
        shoulderGroup = new MotorControllerGroup(leftShoulderMotor, rightShoulderMotorTwo);
    }

    @Override
    public void periodic() {
    }

}