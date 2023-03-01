package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

public class SimpleASubsystem extends SubsystemBase {
    private CANCoder shoulderEncoder;
    private CANCoder elbowEncoder;
    private WPI_TalonFX shoulderLeft;
    private WPI_TalonFX shoulderRight;
    private WPI_TalonFX elbowLeft;
    private WPI_TalonFX elbowRight;

    public SimpleASubsystem() {
        shoulderEncoder = new CANCoder(Constants.SHOULDER_ENCODER_ARM_CAN_ID, Constants.CAN_BUS_NAME_ROBORIO);
        elbowEncoder = new CANCoder(Constants.ELBOW_ENCODER_ARM_CAN_ID, Constants.CAN_BUS_NAME_ROBORIO);
        shoulderLeft = new WPI_TalonFX(Constants.SHOULDER_JOINT_LEFT_MOTOR_CAN_ID,
                Constants.CAN_BUS_NAME_ROBORIO);
        shoulderRight = new WPI_TalonFX(Constants.SHOULDER_JOINT_RIGHT_MOTOR_CAN_ID,
                Constants.CAN_BUS_NAME_ROBORIO);
        elbowLeft = new WPI_TalonFX(Constants.ELBOW_JOINT_LEFT_MOTOR_CAN_ID, Constants.CAN_BUS_NAME_ROBORIO);
        elbowRight = new WPI_TalonFX(Constants.ELBOW_JOINT_RIGHT_MOTOR_CAN_ID, Constants.CAN_BUS_NAME_ROBORIO);
        shoulderLeft.setInverted(true);
        shoulderLeft.follow(shoulderRight);
        elbowLeft.setInverted(true);
        elbowLeft.follow(elbowRight);
        shoulderLeft.setNeutralMode(NeutralMode.Brake);
        shoulderRight.setNeutralMode(NeutralMode.Brake);
        elbowLeft.setNeutralMode(NeutralMode.Brake);
        elbowRight.setNeutralMode(NeutralMode.Brake);
        shoulderStop();
        elbowStop();
    }

    public void shoulderForward() {
        shoulderRight.set(0.2);
    }

    public void shoulderBack() {
        shoulderRight.set(-0.2);
    }

    public void shoulderStop() {
        shoulderRight.set(ControlMode.Position, shoulderRight.getSelectedSensorPosition());
    }

    public void elbowForward() {
        elbowRight.set(0.2);
    }

    public void elbowBack() {
        elbowRight.set(-0.2);
    }

    public void elbowStop() {
        elbowRight.set(ControlMode.Position, elbowRight.getSelectedSensorPosition());
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("shoulderEncoder", shoulderEncoder.getAbsolutePosition());
        SmartDashboard.putNumber("shoulderTrue",
                (shoulderEncoder.getAbsolutePosition() + ArmConstants.SHOULDER_ANGLE_OFFSET + 180) % 360 - 180);
        SmartDashboard.putNumber("elbowEncoder", elbowEncoder.getAbsolutePosition());
        SmartDashboard.putNumber("elbowTrue",
                (elbowEncoder.getAbsolutePosition() + ArmConstants.ELBOW_ANGLE_OFFSET + 180) % 360 - 180);
    }

}
