// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
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
    private WPI_TalonFX m_elbowLeftJoint = new WPI_TalonFX(Constants.ELBOW_JOINT_LEFT_MOTOR_CAN_ID);
    private WPI_TalonFX m_shoulderLeftJoint = new WPI_TalonFX(Constants.SHOULDER_JOINT_LEFT_MOTOR_CAN_ID);
    private WPI_TalonFX m_elbowRightJoint = new WPI_TalonFX(Constants.ELBOW_JOINT_RIGHT_MOTOR_CAN_ID);
    private WPI_TalonFX m_shoulderRightJoint = new WPI_TalonFX(Constants.SHOULDER_JOINT_RIGHT_MOTOR_CAN_ID);

    private CANCoder m_shoulderEncoder = new CANCoder(Constants.SHOULDER_ENCODER_ARM_CAN_ID);
    private CANCoder m_elbowEncoder = new CANCoder(Constants.ELBOW_ENCODER_ARM_CAN_ID);

    private TrapezoidProfile.Constraints elbowConstraints = new TrapezoidProfile.Constraints(
            ArmConstants.SHOULDER_CRUISE,
            ArmConstants.SHOULDER_ACCELERATION);
    private TrapezoidProfile.Constraints shoulderConstraints = new TrapezoidProfile.Constraints(
            ArmConstants.ELBOW_CRUISE,
            ArmConstants.ELBOW_ACCELERATION);

    private ProfiledPIDController m_controllerElbow = new ProfiledPIDController(ArmConstants.GAINS_ELBOW_JOINT.kP,
            ArmConstants.GAINS_ELBOW_JOINT.kI, ArmConstants.GAINS_ELBOW_JOINT.kD, elbowConstraints);
    private ProfiledPIDController m_controllerShoulder = new ProfiledPIDController(ArmConstants.GAINS_SHOULDER_JOINT.kP,
            ArmConstants.GAINS_SHOULDER_JOINT.kI, ArmConstants.GAINS_SHOULDER_JOINT.kD, shoulderConstraints);

    private JointConfig joint_Shoulder = new JointConfig(ArmConstants.SHOULDER_MASS, ArmConstants.SHOULDER_LENGTH,
            ArmConstants.SHOULDER_MOI, ArmConstants.SHOULDER_CGRADIUS, ArmConstants.SHOULDER_MOTOR);
    private JointConfig joint_Elbow = new JointConfig(ArmConstants.ELBOW_MASS, ArmConstants.ELBOW_LENGTH,
            ArmConstants.ELBOW_MOI, ArmConstants.ELBOW_CGRADIUS, ArmConstants.ELBOW_MOTOR);

    private DJArmFeedforward m_doubleJointedFeedForwards = new DJArmFeedforward(joint_Elbow, joint_Shoulder);

    private double m_shoulderSetpoint;
    private double m_elbowSetpoint;

    private RobotContainer container;

    private static final int NO_CHANGE = 500;

    public ArmSubsystem(RobotContainer robotContainer) {
        container = robotContainer;

        m_controllerElbow.setTolerance(1, 1);
        m_controllerShoulder.setTolerance(1, 1);
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

        m_elbowLeftJoint.configSelectedFeedbackSensor(FeedbackDevice.PulseWidthEncodedPosition, 0, ArmConstants.TIMEOUT);
        m_shoulderLeftJoint.configSelectedFeedbackSensor(FeedbackDevice.PulseWidthEncodedPosition, 0, ArmConstants.TIMEOUT);
        m_elbowRightJoint.configSelectedFeedbackSensor(FeedbackDevice.PulseWidthEncodedPosition, 0, ArmConstants.TIMEOUT);
        m_shoulderRightJoint.configSelectedFeedbackSensor(FeedbackDevice.PulseWidthEncodedPosition, 0, ArmConstants.TIMEOUT);

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

        m_shoulderLeftJoint.configForwardSoftLimitThreshold(ArmConstants.FORWARD_SOFT_LIMIT_SHOULDER, ArmConstants.TIMEOUT);
        m_shoulderLeftJoint.configReverseSoftLimitThreshold(ArmConstants.REVERSE_SOFT_LIMIT_SHOULDER, ArmConstants.TIMEOUT);
        m_shoulderRightJoint.configForwardSoftLimitThreshold(ArmConstants.FORWARD_SOFT_LIMIT_SHOULDER, ArmConstants.TIMEOUT);
        m_shoulderRightJoint.configReverseSoftLimitThreshold(ArmConstants.REVERSE_SOFT_LIMIT_SHOULDER, ArmConstants.TIMEOUT);
        m_elbowLeftJoint.configForwardSoftLimitThreshold(ArmConstants.FORWARD_SOFT_LIMIT_ELBOW, ArmConstants.TIMEOUT);
        m_elbowLeftJoint.configReverseSoftLimitThreshold(ArmConstants.REVERSE_SOFT_LIMIT_ELBOW, ArmConstants.TIMEOUT);
        m_elbowRightJoint.configForwardSoftLimitThreshold(ArmConstants.FORWARD_SOFT_LIMIT_ELBOW, ArmConstants.TIMEOUT);
        m_elbowRightJoint.configReverseSoftLimitThreshold(ArmConstants.REVERSE_SOFT_LIMIT_ELBOW, ArmConstants.TIMEOUT);

        m_shoulderEncoder.configMagnetOffset(ArmConstants.SHOULDER_ANGLE_OFFSET);
        m_elbowEncoder.configMagnetOffset(ArmConstants.ELBOW_ANGLE_OFFSET);
        m_shoulderEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
        m_elbowEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
        m_elbowEncoder.configSensorDirection(true);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // m_shoulderLeftJoint.setSelectedSensorPosition(degreesToCTREUnits(getShoulderJointPos()),
        // 0, ArmConstants.TIMEOUT);
        // m_elbowLeftJoint.setSelectedSensorPosition(degreesToCTREUnits(getElbowJointPos()),
        // 0, ArmConstants.TIMEOUT);
        // m_shoulderRightJoint.setSelectedSensorPosition(degreesToCTREUnits(getShoulderJointPos()),
        // 0, ArmConstants.TIMEOUT);
        // m_elbowRightJoint.setSelectedSensorPosition(degreesToCTREUnits(getElbowJointPos()),
        // 0, ArmConstants.TIMEOUT);

        // SmartDashboard.putNumber("Shoulder Setpoint", m_shoulderSetpoint);
        // SmartDashboard.putNumber("Elbow Setpoint", m_elbowSetpoint);
        // SmartDashboard.putNumber("ShoulderEncoder",
        // m_shoulderEncoder.getAbsolutePosition());
        SmartDashboard.putNumber("ShoulderTrue",
                (m_shoulderEncoder.getAbsolutePosition()));
        // SmartDashboard.putNumber("ElbowEncoder",
        // m_elbowEncoder.getAbsolutePosition());
        SmartDashboard.putNumber("ElbowTrue",
                (m_elbowEncoder.getAbsolutePosition()));

        // SmartDashboard.putBoolean("Game Peice", GamePiece.getGamePiece() ==
        // GamePieceType.Cone);

        // if (Constants.TEST_MODE) {
        // SmartDashboard.putNumber("Elbow Angle", getElbowJointDegrees());
        // SmartDashboard.putNumber("Shoulder Angle", getShoulderJointDegrees());
        // SmartDashboard.putNumber("Elbow Angle Uncorrected", getElbowJointPos());
        // SmartDashboard.putNumber("Shoulder Angle Uncorrected",
        // getShoulderJointPos());
        // SmartDashboard.putNumber("Shoulder Percent",
        // m_shoulderLeftJoint.getMotorOutputPercent());
        // SmartDashboard.putNumber("Elbow Percent",
        // m_elbowLeftJoint.getMotorOutputPercent());
        // SmartDashboard.putNumber("Elbow Error",
        // m_controllerElbow.getPositionError());
        // SmartDashboard.putNumber("Shoulder Error",
        // m_controllerShoulder.getPositionError());
        // SmartDashboard.putNumber("Elbow Velocity Setpoint",
        // m_controllerElbow.getPositionError());
        // SmartDashboard.putNumber("Shoulder Velocity Setpoint",
        // m_controllerShoulder.getPositionError());
        // } else {
        // SmartDashboard.clearPersistent("Elbow Angle");
        // SmartDashboard.clearPersistent("Shoulder Angle");
        // SmartDashboard.clearPersistent("Shoulder Percent");
        // SmartDashboard.clearPersistent("Elbow Percent");
        // SmartDashboard.clearPersistent("Shoulder Abs");
        // SmartDashboard.clearPersistent("Elbow Abs");
        // SmartDashboard.clearPersistent("Shoulder Current");
        // SmartDashboard.clearPersistent("Elbow Current");
        // }
    }

    public void reset() {
        System.out.println("Resetting shoulder and elbow to current location");
        m_controllerShoulder.reset(getShoulderJointDegrees());
        m_controllerElbow.reset(getElbowJointDegrees());
        m_shoulderSetpoint = getShoulderJointDegrees();
        m_elbowSetpoint = getElbowJointDegrees();
    }

    public void updateShoulderSetpoint(double setpoint) {
        if (m_shoulderSetpoint != setpoint) {
            if (setpoint < 180 && setpoint > -180) {
                m_shoulderSetpoint = setpoint;
                System.out.println("Shoulder Change ="+setpoint);
            }
        }
    }

    public void updateElbowSetpoint(double setpoint) {
        if (m_elbowSetpoint != setpoint) {
            if (setpoint < 180 && setpoint > -180) {
                m_elbowSetpoint = setpoint;
                System.out.println("Elbow Change ="+setpoint);
            }
        }
    }

    public void updateAllSetpoints(Setpoint setpoint) {
        if (container.getChassisSubsystem().getWantACone()) {
            if (setpoint.m_shoulderCone != NO_CHANGE)
              updateShoulderSetpoint(setpoint.m_shoulderCone);
            if (setpoint.m_elbowCone != NO_CHANGE)
              updateElbowSetpoint(setpoint.m_elbowCone);
        } else if (container.getChassisSubsystem().getWantACube()) {
            if (setpoint.m_shoulderCube != NO_CHANGE)
              updateShoulderSetpoint(setpoint.m_shoulderCube);
            if (setpoint.m_elbowCube != NO_CHANGE)
              updateElbowSetpoint(setpoint.m_elbowCube);
        }
    }

    public boolean atSetpoint(Setpoint setpoint) {
        return false;
    }

    public Vector<N2> calculateFeedforwards() {
        // To set elbow constant, move forearm and bicep to
        // vertical, set to elbow encoder value minus 90 (for horizontal)
        Vector<N2> positionVector = VecBuilder.fill(Math.toRadians(m_elbowSetpoint - (90)),
                // to set shoulder constant, move bicep and forearm to vertical
                // and set to shoulder encoder value
                Math.toRadians(-m_shoulderSetpoint + (180)));

        Vector<N2> velocityVector = VecBuilder.fill(0.0, 0.0);
        Vector<N2> accelVector = VecBuilder.fill(0.0, 0.0);
        Vector<N2> vectorFF = m_doubleJointedFeedForwards.calculate(positionVector, velocityVector, accelVector);
        return vectorFF;
    }

    public void runShoulderProfiled() {
        m_controllerShoulder.setGoal(new TrapezoidProfile.State(m_shoulderSetpoint, 0.0));
        double pidOutput = -m_controllerShoulder.calculate(getShoulderJointDegrees());
        // double ff = -(calculateFeedforwards().get(1, 0)) / 12.0;
        // System.out.println("Shoulder ff" + (ff));
        // System.out.println("Shoulder PID" + pidOutput);
        setPercentOutputShoulder(pidOutput); // may need to negate ff voltage to get desired output
    }

    public void runElbowProfiled() {
        //System.out.println("running elbow="+m_elbowSetpoint);
        m_controllerElbow.setGoal(new TrapezoidProfile.State(m_elbowSetpoint, 0.0));
        double pidOutput = -m_controllerElbow.calculate(getElbowJointDegrees());
        // double ff = -(calculateFeedforwards().get(0, 0)) / 12.0;
        // System.out.println("elbow ff" + (ff));
        // System.out.println("elbow PID" + pidOutput);
        setPercentOutputElbow(pidOutput); // may need to negate ff voltage to get desired output
    }

    public void setToCurrent() {
        m_elbowSetpoint = getElbowJointDegrees();
        m_shoulderSetpoint = getShoulderJointDegrees();
    }

    public boolean shoulderAtSetpoint() {
        return m_controllerShoulder.atSetpoint();
    }

    public boolean elbowAtSetpoint() {
        return m_controllerElbow.atSetpoint();
    }

    public boolean bothJointsAtSetpoint() {
        return shoulderAtSetpoint() && elbowAtSetpoint();
    }

    public void setPercentOutputShoulder(double speed) {
        double t = degreesPerSecondToPower(speed) * 750; //300;
        SmartDashboard.putNumber("shoulder", t);
        //System.out.println("SHOULDER SPEED="+t);
        m_shoulderLeftJoint.set(TalonFXControlMode.PercentOutput, t);
    }

    public void setPercentOutputElbow(double speed) {
        double t = degreesPerSecondToPower(speed) * 300; //120;
        SmartDashboard.putNumber("elbow", t);
        m_elbowLeftJoint.set(TalonFXControlMode.PercentOutput, t);
    }

    public void neutralShoulder() {
        m_shoulderLeftJoint.neutralOutput();
    }

    public void neutralElbow() {
        m_elbowLeftJoint.neutralOutput();
    }

    public double getElbowJointPos() {
        return m_elbowEncoder.getAbsolutePosition();
    }

    public double getShoulderJointPos() {
        return m_shoulderEncoder.getAbsolutePosition();
    }

    public double getElbowJointDegrees() {
        return getElbowJointPos();
    }

    public double getShoulderJointDegrees() {
        return getShoulderJointPos();
    }

    public double degreesToCTREUnits(double degrees) {
        // degrees to CTRE
        return degrees / 360 * 4096;
    }

    public double degreesPerSecondToPower(double degrees) {
        return degrees / 38400;
    }

    // -160 132
    private static final Setpoint BICEP_OUT = new Setpoint(NO_CHANGE, 75, false, NO_CHANGE, 75, false);
    private static final Setpoint FOREARM_HALF1 = new Setpoint(-130, NO_CHANGE, false, -130, NO_CHANGE, false);
    private static final Setpoint FOREARM_HALF2 = new Setpoint(-110, NO_CHANGE, false, -110, NO_CHANGE, false);
    private static final Setpoint FOREARM_HALF3 = new Setpoint(-70, NO_CHANGE, false, -70, NO_CHANGE, false);
    private static final Setpoint FOREARM_HALF4 = new Setpoint(-45, NO_CHANGE, false, -45, NO_CHANGE, false);
    private static final Setpoint FOREARM_HALF5 = new Setpoint(-25, NO_CHANGE, false, -25, NO_CHANGE, false);
    private static final Setpoint FOREARM_FINAL = new Setpoint(-10, 75, false, -10, 75, false);
    // a series of setpoints to get to the mid level scoring 
    public Object midScoreRoutine() {
        updateAllSetpoints(BICEP_OUT); 
        while (!bothJointsAtSetpoint()){
            System.out.println("Moving to BICEP_OUT");
        }
        updateAllSetpoints(FOREARM_HALF1);
        while (!bothJointsAtSetpoint()){
            System.out.println("Moving to FOREARM_HALF");
        }
        updateAllSetpoints(FOREARM_HALF2);
        while (!bothJointsAtSetpoint()){
            System.out.println("Moving to FOREARM_HALF");
        }
        // updateAllSetpoints(FOREARM_HALF3);
        // while (!bothJointsAtSetpoint()){
        //     System.out.println("Moving to FOREARM_HALF");
        // }
        // updateAllSetpoints(FOREARM_HALF4);
        // while (!bothJointsAtSetpoint()){
        //     System.out.println("Moving to FOREARM_HALF");
        // }
        // updateAllSetpoints(FOREARM_HALF5);
        // while (!bothJointsAtSetpoint()){
        //     System.out.println("Moving to FOREARM_HALF");
        // }
        // updateAllSetpoints(FOREARM_FINAL);
        return null;
    }
}
