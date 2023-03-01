// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ChassisSubsystem;
import frc.robot.util.GamePiece;
import frc.robot.util.GamePiece.GamePieceType;

public class ArmSubsystem extends SubsystemBase {
    /** Creates a new ArmSubsystem. */
    private WPI_TalonFX m_elbowLeftJoint = new WPI_TalonFX(Constants.LOWER_JOINT_LEFT_MOTOR_CAN_ID);
    private WPI_TalonFX m_shoulderLeftJoint = new WPI_TalonFX(Constants.UPPER_JOINT_LEFT_MOTOR_CAN_ID);
    private WPI_TalonFX m_elbowRightJoint = new WPI_TalonFX(Constants.LOWER_JOINT_RIGHT_MOTOR_CAN_ID);
    private WPI_TalonFX m_shoulderRightJoint = new WPI_TalonFX(Constants.UPPER_JOINT_RIGHT_MOTOR_CAN_ID);

    private CANCoder m_shoulderEncoder = new CANCoder(Constants.UPPER_ENCODER_ARM_CAN_ID);
    private CANCoder m_elbowEncoder = new CANCoder(Constants.LOWER_ENCODER_ARM_CAN_ID);

    private TrapezoidProfile.Constraints lowerConstraints = new TrapezoidProfile.Constraints(ArmConstants.UPPER_CRUISE,
            ArmConstants.UPPER_ACCELERATION);
    private TrapezoidProfile.Constraints upperConstraints = new TrapezoidProfile.Constraints(ArmConstants.LOWER_CRUISE,
            ArmConstants.LOWER_ACCELERATION);

    private ProfiledPIDController m_controllerLower = new ProfiledPIDController(ArmConstants.GAINS_LOWER_JOINT.kP,
            ArmConstants.GAINS_LOWER_JOINT.kI, ArmConstants.GAINS_LOWER_JOINT.kD, lowerConstraints);
    private ProfiledPIDController m_controllerUpper = new ProfiledPIDController(ArmConstants.GAINS_UPPER_JOINT.kP,
            ArmConstants.GAINS_UPPER_JOINT.kI, ArmConstants.GAINS_UPPER_JOINT.kD, upperConstraints);

    private JointConfig joint_Upper = new JointConfig(ArmConstants.UPPER_MASS, ArmConstants.UPPER_LENGTH,
            ArmConstants.UPPER_MOI, ArmConstants.UPPER_CGRADIUS, ArmConstants.UPPER_MOTOR);
    private JointConfig joint_Lower = new JointConfig(ArmConstants.LOWER_MASS, ArmConstants.LOWER_LENGTH,
            ArmConstants.LOWER_MOI, ArmConstants.LOWER_CGRADIUS, ArmConstants.LOWER_MOTOR);

    private DJArmFeedforward m_doubleJointedFeedForwards = new DJArmFeedforward(joint_Lower, joint_Upper);

    private double m_shoulderSetpoint;
    private double m_elbowSetpoint;

    private RobotContainer container;

    public ArmSubsystem(RobotContainer robotContainer) {
        container = robotContainer;

        m_controllerLower.setTolerance(1, 1);
        m_controllerUpper.setTolerance(1, 1);
        // following
        m_elbowRightJoint.follow(m_elbowLeftJoint);
        m_shoulderRightJoint.follow(m_shoulderLeftJoint);

        // Config Duty Cycle Range for the encoders
        m_elbowEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        m_shoulderEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);

        // Default Motors
        m_elbowLeftJoint.configFactoryDefault(ArmConstants.TIMEOUT);
        m_shoulderLeftJoint.configFactoryDefault(ArmConstants.TIMEOUT);
        m_elbowRightJoint.configFactoryDefault(ArmConstants.TIMEOUT);
        m_shoulderRightJoint.configFactoryDefault(ArmConstants.TIMEOUT);

        // Set Neutral Mode to Brake and NeutralDeadBand to prevent need for intentional
        // stalling
        m_elbowLeftJoint.setNeutralMode(NeutralMode.Brake);
        m_shoulderLeftJoint.setNeutralMode(NeutralMode.Brake);
        m_elbowRightJoint.setNeutralMode(NeutralMode.Brake);
        m_shoulderRightJoint.setNeutralMode(NeutralMode.Brake);

        m_elbowLeftJoint.configNeutralDeadband(ArmConstants.NEUTRAL_DEADBAND);
        m_shoulderLeftJoint.configNeutralDeadband(ArmConstants.NEUTRAL_DEADBAND);
        m_elbowRightJoint.configNeutralDeadband(ArmConstants.NEUTRAL_DEADBAND);
        m_shoulderRightJoint.configNeutralDeadband(ArmConstants.NEUTRAL_DEADBAND);

        m_elbowLeftJoint.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 20, 30, 0.2));
        m_shoulderLeftJoint.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 20, 30, 0.2));
        m_elbowRightJoint.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 20, 30, 0.2));
        m_shoulderRightJoint.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 20, 30, 0.2));

        m_elbowLeftJoint.configSelectedFeedbackSensor(FeedbackDevice.PulseWidthEncodedPosition, 0,
                ArmConstants.TIMEOUT);
        m_shoulderLeftJoint.configSelectedFeedbackSensor(FeedbackDevice.PulseWidthEncodedPosition, 0,
                ArmConstants.TIMEOUT);
        m_elbowRightJoint.configSelectedFeedbackSensor(FeedbackDevice.PulseWidthEncodedPosition, 0,
                ArmConstants.TIMEOUT);
        m_shoulderRightJoint.configSelectedFeedbackSensor(FeedbackDevice.PulseWidthEncodedPosition, 0,
                ArmConstants.TIMEOUT);

        m_shoulderLeftJoint.setInverted(TalonFXInvertType.CounterClockwise);
        m_elbowLeftJoint.setInverted(TalonFXInvertType.CounterClockwise);
        m_shoulderRightJoint.setInverted(TalonFXInvertType.Clockwise);
        m_elbowRightJoint.setInverted(TalonFXInvertType.Clockwise);

        m_elbowLeftJoint.configNominalOutputForward(ArmConstants.NOMINAL_OUTPUT_FORWARD, ArmConstants.TIMEOUT);
        m_elbowLeftJoint.configNominalOutputReverse(ArmConstants.NOMINAL_OUTPUT_REVERSE, ArmConstants.TIMEOUT);
        m_elbowLeftJoint.configPeakOutputForward(ArmConstants.PEAK_OUTPUT_FORWARD, ArmConstants.TIMEOUT);
        m_elbowLeftJoint.configPeakOutputReverse(ArmConstants.PEAK_OUTPUT_REVERSE, ArmConstants.TIMEOUT);

        m_elbowRightJoint.configNominalOutputForward(ArmConstants.NOMINAL_OUTPUT_FORWARD, ArmConstants.TIMEOUT);
        m_elbowRightJoint.configNominalOutputReverse(ArmConstants.NOMINAL_OUTPUT_REVERSE, ArmConstants.TIMEOUT);
        m_elbowRightJoint.configPeakOutputForward(ArmConstants.PEAK_OUTPUT_FORWARD, ArmConstants.TIMEOUT);
        m_elbowRightJoint.configPeakOutputReverse(ArmConstants.PEAK_OUTPUT_REVERSE, ArmConstants.TIMEOUT);

        m_shoulderLeftJoint.configNominalOutputForward(ArmConstants.NOMINAL_OUTPUT_FORWARD, ArmConstants.TIMEOUT);
        m_shoulderLeftJoint.configNominalOutputReverse(ArmConstants.NOMINAL_OUTPUT_REVERSE, ArmConstants.TIMEOUT);
        m_shoulderLeftJoint.configPeakOutputForward(ArmConstants.PEAK_OUTPUT_FORWARD, ArmConstants.TIMEOUT);
        m_shoulderLeftJoint.configPeakOutputReverse(ArmConstants.PEAK_OUTPUT_REVERSE, ArmConstants.TIMEOUT);

        m_shoulderRightJoint.configNominalOutputForward(ArmConstants.NOMINAL_OUTPUT_FORWARD, ArmConstants.TIMEOUT);
        m_shoulderRightJoint.configNominalOutputReverse(ArmConstants.NOMINAL_OUTPUT_REVERSE, ArmConstants.TIMEOUT);
        m_shoulderRightJoint.configPeakOutputForward(ArmConstants.PEAK_OUTPUT_FORWARD, ArmConstants.TIMEOUT);
        m_shoulderRightJoint.configPeakOutputReverse(ArmConstants.PEAK_OUTPUT_REVERSE, ArmConstants.TIMEOUT);

        m_elbowLeftJoint.configVoltageCompSaturation(12, ArmConstants.TIMEOUT);
        m_shoulderLeftJoint.configVoltageCompSaturation(12, ArmConstants.TIMEOUT);
        m_elbowRightJoint.configVoltageCompSaturation(12, ArmConstants.TIMEOUT);
        m_shoulderRightJoint.configVoltageCompSaturation(12, ArmConstants.TIMEOUT);

        m_elbowLeftJoint.configFeedbackNotContinuous(true, ArmConstants.TIMEOUT);
        m_shoulderLeftJoint.configFeedbackNotContinuous(true, ArmConstants.TIMEOUT);
        m_elbowRightJoint.configFeedbackNotContinuous(true, ArmConstants.TIMEOUT);
        m_shoulderRightJoint.configFeedbackNotContinuous(true, ArmConstants.TIMEOUT);

        m_elbowLeftJoint.configForwardSoftLimitEnable(true, ArmConstants.TIMEOUT);
        m_shoulderLeftJoint.configForwardSoftLimitEnable(true, ArmConstants.TIMEOUT);
        m_elbowRightJoint.configForwardSoftLimitEnable(true, ArmConstants.TIMEOUT);
        m_shoulderRightJoint.configForwardSoftLimitEnable(true, ArmConstants.TIMEOUT);

        m_shoulderLeftJoint.configForwardSoftLimitThreshold(ArmConstants.FORWARD_SOFT_LIMIT_UPPER,
                ArmConstants.TIMEOUT);
        m_shoulderLeftJoint.configReverseSoftLimitThreshold(ArmConstants.REVERSE_SOFT_LIMIT_UPPER,
                ArmConstants.TIMEOUT);
        m_shoulderRightJoint.configForwardSoftLimitThreshold(ArmConstants.FORWARD_SOFT_LIMIT_UPPER,
                ArmConstants.TIMEOUT);
        m_shoulderRightJoint.configReverseSoftLimitThreshold(ArmConstants.REVERSE_SOFT_LIMIT_UPPER,
                ArmConstants.TIMEOUT);
        m_elbowLeftJoint.configForwardSoftLimitThreshold(ArmConstants.FORWARD_SOFT_LIMIT_LOWER, ArmConstants.TIMEOUT);
        m_elbowLeftJoint.configReverseSoftLimitThreshold(ArmConstants.REVERSE_SOFT_LIMIT_LOWER, ArmConstants.TIMEOUT);
        m_elbowRightJoint.configForwardSoftLimitThreshold(ArmConstants.FORWARD_SOFT_LIMIT_LOWER, ArmConstants.TIMEOUT);
        m_elbowRightJoint.configReverseSoftLimitThreshold(ArmConstants.REVERSE_SOFT_LIMIT_LOWER, ArmConstants.TIMEOUT);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // m_shoulderLeftJoint.setSelectedSensorPosition(degreesToCTREUnits(getUpperJointPos()),
        // 0, ArmConstants.TIMEOUT);
        // m_elbowLeftJoint.setSelectedSensorPosition(degreesToCTREUnits(getLowerJointPos()),
        // 0, ArmConstants.TIMEOUT);
        // m_shoulderRightJoint.setSelectedSensorPosition(degreesToCTREUnits(getUpperJointPos()),
        // 0, ArmConstants.TIMEOUT);
        // m_elbowRightJoint.setSelectedSensorPosition(degreesToCTREUnits(getLowerJointPos()),
        // 0, ArmConstants.TIMEOUT);

        SmartDashboard.putNumber("Shoulder Setpoint", m_shoulderSetpoint);
        SmartDashboard.putNumber("Elbow Setpoint", m_elbowSetpoint);
        SmartDashboard.putNumber("ShoulderEncoder", m_shoulderEncoder.getAbsolutePosition());
        SmartDashboard.putNumber("ShoulderTrue",
                (m_shoulderEncoder.getAbsolutePosition() + ArmConstants.SHOULDER_ANGLE_OFFSET + 180) % 360 - 180);
        SmartDashboard.putNumber("ElbowEncoder", m_elbowEncoder.getAbsolutePosition());
        SmartDashboard.putNumber("ElbowTrue",
                (m_elbowEncoder.getAbsolutePosition() + ArmConstants.LOWER_ANGLE_OFFSET + 180) % 360 - 180);

        SmartDashboard.putBoolean("Game Peice", GamePiece.getGamePiece() == GamePieceType.Cone);

        // if (Constants.TEST_MODE) {
        // SmartDashboard.putNumber("Lower Angle", getLowerJointDegrees());
        // SmartDashboard.putNumber("Upper Angle", getUpperJointDegrees());
        // SmartDashboard.putNumber("Lower Angle Uncorrected", getLowerJointPos());
        // SmartDashboard.putNumber("Upper Angle Uncorrected", getUpperJointPos());
        // SmartDashboard.putNumber("Upper Percent",
        // m_shoulderLeftJoint.getMotorOutputPercent());
        // SmartDashboard.putNumber("Lower Percent",
        // m_elbowLeftJoint.getMotorOutputPercent());
        // SmartDashboard.putNumber("Lower Error",
        // m_controllerLower.getPositionError());
        // SmartDashboard.putNumber("Upper Error",
        // m_controllerUpper.getPositionError());
        // SmartDashboard.putNumber("Lower Velocity Setpoint",
        // m_controllerLower.getPositionError());
        // SmartDashboard.putNumber("Upper Velocity Setpoint",
        // m_controllerUpper.getPositionError());
        // } else {
        // SmartDashboard.clearPersistent("Lower Angle");
        // SmartDashboard.clearPersistent("Upper Angle");
        // SmartDashboard.clearPersistent("Upper Percent");
        // SmartDashboard.clearPersistent("Lower Percent");
        // SmartDashboard.clearPersistent("Upper Abs");
        // SmartDashboard.clearPersistent("Lower Abs");
        // SmartDashboard.clearPersistent("Upper Current");
        // SmartDashboard.clearPersistent("Lower Current");
        // }
    }

    public void reset() {
        m_controllerUpper.reset(getUpperJointDegrees());
        m_controllerLower.reset(getLowerJointDegrees());
        m_shoulderSetpoint = getUpperJointDegrees();
        m_elbowSetpoint = getLowerJointDegrees();
    }

    public void updateUpperSetpoint(double setpoint) {
        if (m_shoulderSetpoint != setpoint) {
            if (setpoint < 180 && setpoint > -180) {
                m_shoulderSetpoint = setpoint;
            }
        }
    }

    public void updateLowerSetpoint(double setpoint) {
        if (m_elbowSetpoint != setpoint) {
            if (setpoint < 180 && setpoint > -180) {
                m_elbowSetpoint = setpoint;
            }
        }
    }

    public void updateAllSetpoints(Setpoint setpoint) {
        if (container.getChassisSubsystem().getWantACone()) {
            updateUpperSetpoint(setpoint.m_shoulderCone);
            updateLowerSetpoint(setpoint.m_elbowCone);
        } else if (container.getChassisSubsystem().getWantACube()) {
            updateUpperSetpoint(setpoint.m_shoulderCube);
            updateLowerSetpoint(setpoint.m_elbowCube);
        }
    }

    public Vector<N2> calculateFeedforwards() {
        // To set lower constant, move lower and upper arms to
        // vertical, set to lower encoder value minus 90 (for horizontal)
        Vector<N2> positionVector = VecBuilder.fill(Math.toRadians(m_elbowSetpoint - (90)),
                // to set upper constant, move upper arm and lower arms to vertical
                // and set to upper encoder value
                Math.toRadians(-m_shoulderSetpoint + (180)));

        Vector<N2> velocityVector = VecBuilder.fill(0.0, 0.0);
        Vector<N2> accelVector = VecBuilder.fill(0.0, 0.0);
        Vector<N2> vectorFF = m_doubleJointedFeedForwards.calculate(positionVector, velocityVector, accelVector);
        return vectorFF;
    }

    public void runUpperProfiled() {
        m_controllerUpper.setGoal(new TrapezoidProfile.State(m_shoulderSetpoint, 0.0));
        double pidOutput = -m_controllerUpper.calculate(getUpperJointDegrees());
        // double ff = -(calculateFeedforwards().get(1, 0)) / 12.0;
        // System.out.println("upper ff" + (ff));
        // System.out.println("Upper PID" + pidOutput);
        setPercentOutputUpper(pidOutput); // may need to negate ff voltage to get desired output
    }

    public void runLowerProfiled() {
        m_controllerLower.setGoal(new TrapezoidProfile.State(m_elbowSetpoint, 0.0));
        double pidOutput = -m_controllerLower.calculate(getLowerJointDegrees());
        // double ff = -(calculateFeedforwards().get(0, 0)) / 12.0;
        // System.out.println("lower ff" + (ff));
        // System.out.println("Lower PID" + pidOutput);
        setPercentOutputLower(pidOutput); // may need to negate ff voltage to get desired output
    }

    public void setToCurrent() {
        m_elbowSetpoint = getLowerJointDegrees();
        m_shoulderSetpoint = getUpperJointDegrees();
    }

    public boolean upperAtSetpoint() {
        return m_controllerUpper.atSetpoint();
    }

    public boolean lowerAtSetpoint() {
        return m_controllerLower.atSetpoint();
    }

    public boolean bothJointsAtSetpoint() {
        return upperAtSetpoint() && lowerAtSetpoint();
    }

    public void setPercentOutputUpper(double speed) {
        // m_shoulderLeftJoint.set(TalonFXControlMode.PercentOutput, speed);
        System.out.println("shoulder-" + speed);
    }

    public void setPercentOutputLower(double speed) {
        // m_elbowLeftJoint.set(TalonFXControlMode.PercentOutput, speed);
        System.out.println("elbow-" + speed);
    }

    public void neutralUpper() {
        m_shoulderLeftJoint.neutralOutput();
    }

    public void neutralLower() {
        m_elbowLeftJoint.neutralOutput();
    }

    public double getLowerJointPos() {
        return m_elbowEncoder.getAbsolutePosition();
    }

    public double getUpperJointPos() {
        return m_shoulderEncoder.getAbsolutePosition();
    }

    public double getLowerJointDegrees() {
        return (getLowerJointPos() + ArmConstants.LOWER_ANGLE_OFFSET + 180) % 360 - 180;
    }

    public double getUpperJointDegrees() {
        return (getUpperJointPos() + ArmConstants.SHOULDER_ANGLE_OFFSET + 180) % 360 - 180;
    }

    public double degreesToCTREUnits(double degrees) {
        // degrees to CTRE
        return degrees / 360 * 4096;
    }
}
