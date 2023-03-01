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
    private CANCoder upperEncoder;
    private CANCoder lowerEncoder;
    private WPI_TalonFX upperLeft;
    private WPI_TalonFX upperRight;
    private WPI_TalonFX lowerLeft;
    private WPI_TalonFX lowerRight;

    public SimpleASubsystem() {
        upperEncoder = new CANCoder(Constants.UPPER_ENCODER_ARM_CAN_ID, Constants.CAN_BUS_NAME_ROBORIO);
        lowerEncoder = new CANCoder(Constants.LOWER_ENCODER_ARM_CAN_ID, Constants.CAN_BUS_NAME_ROBORIO);
        upperLeft = new WPI_TalonFX(Constants.UPPER_JOINT_LEFT_MOTOR_CAN_ID,
                Constants.CAN_BUS_NAME_ROBORIO);
        upperRight = new WPI_TalonFX(Constants.UPPER_JOINT_RIGHT_MOTOR_CAN_ID,
                Constants.CAN_BUS_NAME_ROBORIO);
        lowerLeft = new WPI_TalonFX(Constants.LOWER_JOINT_LEFT_MOTOR_CAN_ID, Constants.CAN_BUS_NAME_ROBORIO);
        lowerRight = new WPI_TalonFX(Constants.LOWER_JOINT_RIGHT_MOTOR_CAN_ID, Constants.CAN_BUS_NAME_ROBORIO);
        upperLeft.setInverted(true);
        upperLeft.follow(upperRight);
        lowerLeft.setInverted(true);
        lowerLeft.follow(lowerRight);
        upperLeft.setNeutralMode(NeutralMode.Brake);
        upperRight.setNeutralMode(NeutralMode.Brake);
        lowerLeft.setNeutralMode(NeutralMode.Brake);
        lowerRight.setNeutralMode(NeutralMode.Brake);
        upperStop();
        lowerStop();
    }

    public void upperForward() {
        upperRight.set(0.2);
    }

    public void upperBack() {
        upperRight.set(-0.2);
    }

    public void upperStop() {
        upperRight.set(ControlMode.Position, upperRight.getSelectedSensorPosition());
    }

    public void lowerForward() {
        lowerRight.set(0.2);
    }

    public void lowerBack() {
        lowerRight.set(-0.2);
    }

    public void lowerStop() {
        lowerRight.set(ControlMode.Position, lowerRight.getSelectedSensorPosition());
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("upperEncoder", upperEncoder.getAbsolutePosition());
        SmartDashboard.putNumber("upperTrue",
                (upperEncoder.getAbsolutePosition() + ArmConstants.SHOULDER_ANGLE_OFFSET + 180) % 360 - 180);
        SmartDashboard.putNumber("lowerEncoder", lowerEncoder.getAbsolutePosition());
        SmartDashboard.putNumber("lowerTrue",
                (lowerEncoder.getAbsolutePosition() + ArmConstants.LOWER_ANGLE_OFFSET + 180) % 360 - 180);
    }

}
