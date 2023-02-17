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
import frc.robot.Constants.ArmConstants;
import frc.robot.util.GamePiece;
import frc.robot.util.GamePiece.GamePieceType;

public class ArmSubsystem extends SubsystemBase {
    /** Creates a new ArmSubsystem. */
    private WPI_TalonFX m_lowerLeftJoint = new WPI_TalonFX(Constants.LOWER_JOINT_LEFT_MOTOR_CAN_ID);
    private WPI_TalonFX m_upperLeftJoint = new WPI_TalonFX(Constants.UPPER_JOINT_LEFT_MOTOR_CAN_ID);
    private WPI_TalonFX m_lowerRightJoint = new WPI_TalonFX(Constants.LOWER_JOINT_RIGHT_MOTOR_CAN_ID);
    private WPI_TalonFX m_upperRightJoint = new WPI_TalonFX(Constants.UPPER_JOINT_RIGHT_MOTOR_CAN_ID);

    private CANCoder m_upperEncoder = new CANCoder(Constants.UPPER_ENCODER_ARM_CAN_ID);
    private CANCoder m_lowerEncoder = new CANCoder(Constants.LOWER_ENCODER_ARM_CAN_ID);

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

    private double m_upperSetpoint;
    private double m_lowerSetpoint;

    public ArmSubsystem() {
        // following
        m_lowerRightJoint.follow(m_lowerLeftJoint);
        m_upperRightJoint.follow(m_upperLeftJoint);

        // Config Duty Cycle Range for the encoders
        m_lowerEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        m_upperEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);

        // Default Motors
        m_lowerLeftJoint.configFactoryDefault(ArmConstants.TIMEOUT);
        m_upperLeftJoint.configFactoryDefault(ArmConstants.TIMEOUT);
        m_lowerRightJoint.configFactoryDefault(ArmConstants.TIMEOUT);
        m_upperRightJoint.configFactoryDefault(ArmConstants.TIMEOUT);

        // Set Neutral Mode to Brake and NeutralDeadBand to prevent need for intentional
        // stalling
        m_lowerLeftJoint.setNeutralMode(NeutralMode.Brake);
        m_upperLeftJoint.setNeutralMode(NeutralMode.Brake);
        m_lowerRightJoint.setNeutralMode(NeutralMode.Brake);
        m_upperRightJoint.setNeutralMode(NeutralMode.Brake);

        m_lowerLeftJoint.configNeutralDeadband(ArmConstants.NEUTRAL_DEADBAND);
        m_upperLeftJoint.configNeutralDeadband(ArmConstants.NEUTRAL_DEADBAND);
        m_lowerRightJoint.configNeutralDeadband(ArmConstants.NEUTRAL_DEADBAND);
        m_upperRightJoint.configNeutralDeadband(ArmConstants.NEUTRAL_DEADBAND);

        m_lowerLeftJoint.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 20, 30, 0.2));
        m_upperLeftJoint.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 20, 30, 0.2));
        m_lowerRightJoint.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 20, 30, 0.2));
        m_upperRightJoint.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 20, 30, 0.2));

        m_lowerLeftJoint.configSelectedFeedbackSensor(FeedbackDevice.PulseWidthEncodedPosition, 0,
                ArmConstants.TIMEOUT);
        m_upperLeftJoint.configSelectedFeedbackSensor(FeedbackDevice.PulseWidthEncodedPosition, 0,
                ArmConstants.TIMEOUT);
        m_lowerRightJoint.configSelectedFeedbackSensor(FeedbackDevice.PulseWidthEncodedPosition, 0,
                ArmConstants.TIMEOUT);
        m_upperRightJoint.configSelectedFeedbackSensor(FeedbackDevice.PulseWidthEncodedPosition, 0,
                ArmConstants.TIMEOUT);

        m_upperLeftJoint.setInverted(TalonFXInvertType.CounterClockwise);
        m_lowerLeftJoint.setInverted(TalonFXInvertType.CounterClockwise);
        m_upperRightJoint.setInverted(TalonFXInvertType.Clockwise);
        m_lowerRightJoint.setInverted(TalonFXInvertType.Clockwise);

        m_lowerLeftJoint.configNominalOutputForward(ArmConstants.NOMINAL_OUTPUT_FORWARD, ArmConstants.TIMEOUT);
        m_lowerLeftJoint.configNominalOutputReverse(ArmConstants.NOMINAL_OUTPUT_REVERSE, ArmConstants.TIMEOUT);
        m_lowerLeftJoint.configPeakOutputForward(ArmConstants.PEAK_OUTPUT_FORWARD, ArmConstants.TIMEOUT);
        m_lowerLeftJoint.configPeakOutputReverse(ArmConstants.PEAK_OUTPUT_REVERSE, ArmConstants.TIMEOUT);

        m_lowerRightJoint.configNominalOutputForward(ArmConstants.NOMINAL_OUTPUT_FORWARD, ArmConstants.TIMEOUT);
        m_lowerRightJoint.configNominalOutputReverse(ArmConstants.NOMINAL_OUTPUT_REVERSE, ArmConstants.TIMEOUT);
        m_lowerRightJoint.configPeakOutputForward(ArmConstants.PEAK_OUTPUT_FORWARD, ArmConstants.TIMEOUT);
        m_lowerRightJoint.configPeakOutputReverse(ArmConstants.PEAK_OUTPUT_REVERSE, ArmConstants.TIMEOUT);

        m_upperLeftJoint.configNominalOutputForward(ArmConstants.NOMINAL_OUTPUT_FORWARD, ArmConstants.TIMEOUT);
        m_upperLeftJoint.configNominalOutputReverse(ArmConstants.NOMINAL_OUTPUT_REVERSE, ArmConstants.TIMEOUT);
        m_upperLeftJoint.configPeakOutputForward(ArmConstants.PEAK_OUTPUT_FORWARD, ArmConstants.TIMEOUT);
        m_upperLeftJoint.configPeakOutputReverse(ArmConstants.PEAK_OUTPUT_REVERSE, ArmConstants.TIMEOUT);

        m_upperRightJoint.configNominalOutputForward(ArmConstants.NOMINAL_OUTPUT_FORWARD, ArmConstants.TIMEOUT);
        m_upperRightJoint.configNominalOutputReverse(ArmConstants.NOMINAL_OUTPUT_REVERSE, ArmConstants.TIMEOUT);
        m_upperRightJoint.configPeakOutputForward(ArmConstants.PEAK_OUTPUT_FORWARD, ArmConstants.TIMEOUT);
        m_upperRightJoint.configPeakOutputReverse(ArmConstants.PEAK_OUTPUT_REVERSE, ArmConstants.TIMEOUT);

        m_lowerLeftJoint.configVoltageCompSaturation(12, ArmConstants.TIMEOUT);
        m_upperLeftJoint.configVoltageCompSaturation(12, ArmConstants.TIMEOUT);
        m_lowerRightJoint.configVoltageCompSaturation(12, ArmConstants.TIMEOUT);
        m_upperRightJoint.configVoltageCompSaturation(12, ArmConstants.TIMEOUT);

        m_lowerLeftJoint.configFeedbackNotContinuous(true, ArmConstants.TIMEOUT);
        m_upperLeftJoint.configFeedbackNotContinuous(true, ArmConstants.TIMEOUT);
        m_lowerRightJoint.configFeedbackNotContinuous(true, ArmConstants.TIMEOUT);
        m_upperRightJoint.configFeedbackNotContinuous(true, ArmConstants.TIMEOUT);

        m_lowerLeftJoint.configForwardSoftLimitEnable(true, ArmConstants.TIMEOUT);
        m_upperLeftJoint.configForwardSoftLimitEnable(true, ArmConstants.TIMEOUT);
        m_lowerRightJoint.configForwardSoftLimitEnable(true, ArmConstants.TIMEOUT);
        m_upperRightJoint.configForwardSoftLimitEnable(true, ArmConstants.TIMEOUT);

        m_upperLeftJoint.configForwardSoftLimitThreshold(ArmConstants.FORWARD_SOFT_LIMIT_UPPER, ArmConstants.TIMEOUT);
        m_upperLeftJoint.configReverseSoftLimitThreshold(ArmConstants.REVERSE_SOFT_LIMIT_UPPER, ArmConstants.TIMEOUT);
        m_upperRightJoint.configForwardSoftLimitThreshold(ArmConstants.FORWARD_SOFT_LIMIT_UPPER, ArmConstants.TIMEOUT);
        m_upperRightJoint.configReverseSoftLimitThreshold(ArmConstants.REVERSE_SOFT_LIMIT_UPPER, ArmConstants.TIMEOUT);
        m_lowerLeftJoint.configForwardSoftLimitThreshold(ArmConstants.FORWARD_SOFT_LIMIT_LOWER, ArmConstants.TIMEOUT);
        m_lowerLeftJoint.configReverseSoftLimitThreshold(ArmConstants.REVERSE_SOFT_LIMIT_LOWER, ArmConstants.TIMEOUT);
        m_lowerRightJoint.configForwardSoftLimitThreshold(ArmConstants.FORWARD_SOFT_LIMIT_LOWER, ArmConstants.TIMEOUT);
        m_lowerRightJoint.configReverseSoftLimitThreshold(ArmConstants.REVERSE_SOFT_LIMIT_LOWER, ArmConstants.TIMEOUT);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        m_upperLeftJoint.setSelectedSensorPosition(degreesToCTREUnits(getUpperJointPos()), 0, ArmConstants.TIMEOUT);
        m_lowerLeftJoint.setSelectedSensorPosition(degreesToCTREUnits(getLowerJointPos()), 0, ArmConstants.TIMEOUT);
        m_upperRightJoint.setSelectedSensorPosition(degreesToCTREUnits(getUpperJointPos()), 0, ArmConstants.TIMEOUT);
        m_lowerRightJoint.setSelectedSensorPosition(degreesToCTREUnits(getLowerJointPos()), 0, ArmConstants.TIMEOUT);

        SmartDashboard.putNumber("Upper Setpoint", m_upperSetpoint);
        SmartDashboard.putNumber("Lower Setpoint", m_lowerSetpoint);

        SmartDashboard.putBoolean("Game Peice", GamePiece.getGamePiece() == GamePieceType.Cone);

        if (Constants.TEST_MODE) {
            SmartDashboard.putNumber("Lower Angle", getLowerJointDegrees());
            SmartDashboard.putNumber("Upper Angle", getUpperJointDegrees());
            SmartDashboard.putNumber("Lower Angle Uncorrected", getLowerJointPos());
            SmartDashboard.putNumber("Upper Angle Uncorrected", getUpperJointPos());
            SmartDashboard.putNumber("Upper Percent", m_upperLeftJoint.getMotorOutputPercent());
            SmartDashboard.putNumber("Lower Percent", m_lowerLeftJoint.getMotorOutputPercent());
            SmartDashboard.putNumber("Lower Error", m_controllerLower.getPositionError());
            SmartDashboard.putNumber("Upper Error", m_controllerUpper.getPositionError());
            SmartDashboard.putNumber("Lower Velocity Setpoint", m_controllerLower.getPositionError());
            SmartDashboard.putNumber("Upper Velocity Setpoint", m_controllerUpper.getPositionError());
        } else {
            SmartDashboard.clearPersistent("Lower Angle");
            SmartDashboard.clearPersistent("Upper Angle");
            SmartDashboard.clearPersistent("Upper Percent");
            SmartDashboard.clearPersistent("Lower Percent");
            SmartDashboard.clearPersistent("Upper Abs");
            SmartDashboard.clearPersistent("Lower Abs");
            SmartDashboard.clearPersistent("Upper Current");
            SmartDashboard.clearPersistent("Lower Current");
        }
    }

    public void reset() {
        m_controllerUpper.reset(getUpperJointDegrees());
        m_controllerLower.reset(getLowerJointDegrees());
        m_upperSetpoint = getUpperJointDegrees();
        m_lowerSetpoint = getLowerJointDegrees();

    }

    public void updateUpperSetpoint(double setpoint) {
        if (m_upperSetpoint != setpoint) {
            if (setpoint < 360 && setpoint > 0) {
                m_upperSetpoint = setpoint;
            }
        }
    }

    public void updateLowerSetpoint(double setpoint) {
        if (m_lowerSetpoint != setpoint) {
            if (setpoint < 360 && setpoint > 0) {
                m_lowerSetpoint = setpoint;
            }
        }
    }

    public void updateAllSetpoints(Setpoint setpoint) {
        if (GamePiece.getGamePiece() == GamePieceType.Cone) {
            updateUpperSetpoint(setpoint.m_upperCone);
            updateLowerSetpoint(setpoint.m_lowerCone);
        } else if (GamePiece.getGamePiece() == GamePieceType.Cube) {
            updateUpperSetpoint(setpoint.m_upperCube);
            updateLowerSetpoint(setpoint.m_lowerCube);
        }
    }

    public Vector<N2> calculateFeedforwards() {
        // To set lower constant, move lower and upper arms to
        // vertical, set to lower encoder value minus 90 (for horizontal)
        Vector<N2> positionVector = VecBuilder.fill(Math.toRadians(m_lowerSetpoint - (90)),
                // to set upper constant, move upper arm and lower arms to vertical
                // and set to upper encoder value
                Math.toRadians(-m_upperSetpoint + (180)));

        Vector<N2> velocityVector = VecBuilder.fill(0.0, 0.0);
        Vector<N2> accelVector = VecBuilder.fill(0.0, 0.0);
        Vector<N2> vectorFF = m_doubleJointedFeedForwards.calculate(positionVector, velocityVector, accelVector);
        return vectorFF;
    }

    public void runUpperProfiled() {
        m_controllerUpper.setGoal(new TrapezoidProfile.State(m_upperSetpoint, 0.0));
        double pidOutput = -m_controllerUpper.calculate(getUpperJointDegrees());
        // double ff = -(calculateFeedforwards().get(1, 0)) / 12.0;
        // System.out.println("upper ff" + (ff));
        // System.out.println("Upper PID" + pidOutput);
        setPercentOutputUpper(pidOutput); // may need to negate ff voltage to get desired output
    }

    public void runLowerProfiled() {
        m_controllerLower.setGoal(new TrapezoidProfile.State(m_lowerSetpoint, 0.0));
        double pidOutput = -m_controllerLower.calculate(getLowerJointDegrees());
        // double ff = -(calculateFeedforwards().get(0, 0)) / 12.0;
        // System.out.println("lower ff" + (ff));
        // System.out.println("Lower PID" + pidOutput);
        setPercentOutputLower(pidOutput); // may need to negate ff voltage to get desired output
    }

    public void setToCurrent() {
        m_lowerSetpoint = getLowerJointDegrees();
        m_upperSetpoint = getUpperJointDegrees();
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
        m_upperLeftJoint.set(TalonFXControlMode.PercentOutput, speed);
    }

    public void setPercentOutputLower(double speed) {
        m_lowerLeftJoint.set(TalonFXControlMode.PercentOutput, speed);
    }

    public void neutralUpper() {
        m_upperLeftJoint.neutralOutput();
    }

    public void neutralLower() {
        m_lowerLeftJoint.neutralOutput();
    }

    public double getLowerJointPos() {
        return m_lowerEncoder.getAbsolutePosition();
    }

    public double getUpperJointPos() {
        return m_upperEncoder.getAbsolutePosition();
    }

    public double getLowerJointDegrees() {
        return (getLowerJointPos() + ArmConstants.LOWER_ANGLE_OFFSET + 180) % 360 - 180;
    }

    public double getUpperJointDegrees() {
        return (getUpperJointPos() + ArmConstants.UPPER_ANGLE_OFFSET + 180) % 360 - 180;
    }

    public double degreesToCTREUnits(double degrees) {
        // degrees to CTRE
        return degrees / 360 * 4096;
    }
}
