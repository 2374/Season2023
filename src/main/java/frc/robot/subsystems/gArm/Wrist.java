// package frc.robot.subsystems.gArm;

// import com.ctre.phoenix.motorcontrol.NeutralMode;
// import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
// import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
// import com.ctre.phoenix.sensors.CANCoder;
// // import com.revrobotics.AbsoluteEncoder;
// // import com.revrobotics.CANSparkMax;

// // import com.revrobotics.SparkMaxAbsoluteEncoder;
// import edu.wpi.first.math.controller.ArmFeedforward;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.geometry.Rotation2d;
// // import edu.wpi.first.math.util.Units;
// // import edu.wpi.first.wpilibj.DigitalInput;
// // import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import frc.robot.Constants;
// // import frc.robot.RobotContainer;

// public class Wrist {
// protected final Arm arm;
// private WPI_TalonFX m_elbowLeftJoint = new
// WPI_TalonFX(Constants.ELBOW_JOINT_LEFT_MOTOR_CAN_ID);
// private WPI_TalonFX m_elbowRightJoint = new
// WPI_TalonFX(Constants.ELBOW_JOINT_RIGHT_MOTOR_CAN_ID);

// private CANCoder m_elbowEncoder = new
// CANCoder(Constants.ELBOW_ENCODER_ARM_CAN_ID);

// // private final CANSparkMax motor = new CANSparkMax(Constants.Wrist.ID,
// CANSparkMax.MotorType.kBrushless); ELBOW_JOINT_LEFT_MOTOR_CAN_ID
// // private final DigitalInput limitSwitch = new
// DigitalInput(Constants.Wrist.LIMIT_SWITCH);
// // private final AbsoluteEncoder encoder =
// motor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);

// protected final PIDController pidController = new
// PIDController(Constants.Wrist.KP, Constants.Wrist.KI, Constants.Wrist.KD);
// protected final ArmFeedforward feedforward = new
// ArmFeedforward(Constants.Wrist.KS, Constants.Wrist.KG, Constants.Wrist.KV,
// Constants.Wrist.KA);

// public Wrist(Arm arm) {
// this.arm = arm;

// // motor.restoreFactoryDefaults();
// m_elbowRightJoint.follow(m_elbowLeftJoint);

// // motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
// m_elbowLeftJoint.setNeutralMode(NeutralMode.Brake);
// m_elbowRightJoint.setNeutralMode(NeutralMode.Brake);
// // motor.getEncoder().setPositionConversionFactor(Units.rotationsToRadians(1)
// * Constants.Wrist.GEAR_RATIO);
// // motor.getEncoder().setVelocityConversionFactor(Units.rotationsToRadians(1)
// * Constants.Wrist.GEAR_RATIO / 60.0);
// //encoder.setPositionConversionFactor(Units.rotationsToRadians(1));
// //encoder.setVelocityConversionFactor(Units.rotationsToRadians(1) / 60.0);

// m_elbowLeftJoint.configSupplyCurrentLimit(new
// SupplyCurrentLimitConfiguration(true, 40, 40, 0.2));
// m_elbowRightJoint.configSupplyCurrentLimit(new
// SupplyCurrentLimitConfiguration(true, 40, 40, 0.2));
// m_elbowRightJoint.setInverted(true);
// // encoder.setInverted(false);
// // m_elbowEncoder.setInverted(false);

// // zeroEncoder();

// //RobotContainer.armTab.add("Wrist PID",
// pidController).withWidget(BuiltInWidgets.kPIDController);
// }

// public Rotation2d getAngle() {
// return Rotation2d.fromDegrees(m_elbowEncoder.getAbsolutePosition());
// }

// public Rotation2d getAbsoluteAngle() {
// return Rotation2d.fromDegrees(m_elbowEncoder.getAbsolutePosition());
// // return Rotation2d.fromRadians(encoder.getPosition() <
// Constants.Wrist.LIMIT_SWITCH_OFFSET.getRadians() ? encoder.getPosition() :
// encoder.getPosition() - Math.PI * 2);
// }

// public void followShoulderWithVelocity(Rotation2d velocity) {
// // if(arm.getTarget() == Arm.State.Stowed && arm.getRollerState() ==
// Rollers.State.Off) {
// // velocity = Rotation2d.fromRadians(velocity.getRadians() + 1);
// // }
// if(arm.getTarget() == Arm.State.Stowed ) {
// velocity = Rotation2d.fromRadians(velocity.getRadians() + 1);
// }
// runWithSetpoint(getSetPosition(), velocity);
// }

// public void runWithVelocity(Rotation2d velocity) {
// // runWithSetpoint(Rotation2d.fromRadians(encoder.getPosition()), velocity);
// runWithSetpoint(Rotation2d.fromRadians(m_elbowEncoder.getAbsolutePosition()),
// velocity);
// }

// public void runWithSetpoint(Rotation2d position, Rotation2d velocity) {
// if(velocity.getRadians() != 0) {
// Rotation2d minAngle = getWristAngleFromHeight(0.4);
// if(minAngle != null) {
// position = Rotation2d.fromRadians(Math.max(position.getRadians(),
// minAngle.getRadians()));
// }
// }

// velocity = Rotation2d.fromRadians(velocity.getRadians() +
// pidController.calculate(arm.getWristAngle().getRadians(),
// position.getRadians()));

// SmartDashboard.putNumber("Wrist Setpoint", position.getRadians());

// // if (isLimitSwitchPressed() && velocity.getRadians() >= 0) {
// // motor.set(0);
// // return;
// // }

// // motor.setVoltage(feedforward.calculate(arm.getWristAngle().getRadians(),
// velocity.getRadians()));
// m_elbowLeftJoint.setVoltage(feedforward.calculate(arm.getWristAngle().getRadians(),
// velocity.getRadians()));
// }

// public Rotation2d getSetPosition() {
// return arm.getTargetWristAngle();
// }

// public Rotation2d getWristAngleFromHeight(double height) {
// double jointHeight = Constants.Arm.PIVOT_HEIGHT -
// Constants.Arm.HUMERUS_LENGTH * Math.cos(arm.getShoulderAngle().getRadians());
// if(Math.abs((height - jointHeight) / Constants.Arm.MANIPULATOR_LENGTH) > 1) {
// return null;
// }
// return Rotation2d.fromRadians(Math.asin((height - jointHeight) /
// Constants.Arm.MANIPULATOR_LENGTH));
// }

// // public boolean isLimitSwitchPressed() {
// // return !limitSwitch.get();
// // }

// public void periodic() {
// // SmartDashboard.putBoolean("Wrist Limit Switch", limitSwitch.get());
// // SmartDashboard.putNumber("Wrist Angle",
// Units.radiansToDegrees(encoder.getPosition()));
// SmartDashboard.putNumber("Wrist Set Point",
// arm.getTargetWristAngle().getDegrees());
// SmartDashboard.putNumber("Wrist Relative", arm.getWristAngle().getDegrees());

// // if(isLimitSwitchPressed()) {
// // motor.getEncoder().setPosition(getAbsoluteAngle().getRadians());
// // }
// }

// // private void zeroEncoder() {
// // motor.getEncoder().setPosition(encoder.getPosition());
// // }
// }
