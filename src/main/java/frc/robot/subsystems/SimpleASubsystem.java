package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

public class SimpleASubsystem extends SubsystemBase {
    private CANCoder encoder;
    private WPI_TalonFX left;
    private WPI_TalonFX right;

    public SimpleASubsystem() {
        encoder = new CANCoder(Constants.UPPER_ENCODER_ARM_CAN_ID);
        left = new WPI_TalonFX(Constants.UPPER_JOINT_LEFT_MOTOR_CAN_ID, Constants.CAN_BUS_NAME_ROBORIO);
        right = new WPI_TalonFX(Constants.UPPER_JOINT_RIGHT_MOTOR_CAN_ID, Constants.CAN_BUS_NAME_ROBORIO);
        left.setInverted(true);
        left.follow(right);
        left.setNeutralMode(NeutralMode.Brake);
        right.setNeutralMode(NeutralMode.Brake);
    }

    public void f() {
        right.set(0.2);
    }

    public void b() {
        right.set(-0.2);
    }

    public void s() {
        right.stopMotor();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("encoder", encoder.getAbsolutePosition());
        SmartDashboard.putNumber("true", encoder.getAbsolutePosition() + ArmConstants.UPPER_ANGLE_OFFSET);
    }

}
