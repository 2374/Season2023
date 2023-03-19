package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.arm.ArmDefault;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.util.AutonomousChooser;
import frc.robot.util.AutonomousTrajectories;

public class RobotContainer {
    private final ChassisSubsystem m_ChassisSubsystem;
    private final DrivetrainSubsystem m_DrivetrainSubsystem;
    private final ArmSubsystem m_ArmSubsystem;
    private final ManipulatorSubsystem m_ManipulatorSubsystem;
    // private final SimpleASubsystem m_ASubsystem = new SimpleASubsystem();

    private final AutonomousChooser autonomousChooser = new AutonomousChooser(
            new AutonomousTrajectories(DrivetrainSubsystem.TRAJECTORY_CONSTRAINTS));

    private final XboxController m_driveController = new XboxController(Constants.CONTROLLER_USB_PORT_DRIVER);
    private final XboxController m_operatorController = new XboxController(Constants.CONTROLLER_USB_PORT_OPERATOR);

    private SlewRateLimiter xLimiter = new SlewRateLimiter(6);
    private SlewRateLimiter yLimiter = new SlewRateLimiter(6);

    private boolean slow = false;
    private boolean turbo = false;

    /**
     * The robot container. Need I say more?
     */
    public RobotContainer() {
        m_ChassisSubsystem = new ChassisSubsystem();
        m_DrivetrainSubsystem = new DrivetrainSubsystem(this);
        if (!m_ChassisSubsystem.isTestRobot()) {
            m_ArmSubsystem = new ArmSubsystem(this);
            resetArm();
            m_ManipulatorSubsystem = new ManipulatorSubsystem(this);
        } else {
            m_ArmSubsystem = null;
            m_ManipulatorSubsystem = null;
        }
        System.out.println("container created");

        // resetDrive();

        configureButtonBindings();
        configureShuffleBoard();
    }

    /**
     * Reset the default drive command
     */
    public void resetDrive() {
        m_DrivetrainSubsystem.setDefaultCommand(
                new DefaultDriveCommand(m_DrivetrainSubsystem, this::getForwardInput, this::getStrafeInput,
                        this::getRotationInput));
    }

    /**
     * Reset the default arm command
     */
    public void resetArm() {
        if (!m_ChassisSubsystem.isTestRobot()) {
            m_ArmSubsystem.reset();
            m_ArmSubsystem.setDefaultCommand(new ArmDefault(m_ArmSubsystem));
        }
    }

    /**
     * Get the main controller
     * 
     * @return The main controller
     */
    public XboxController getMainController() {
        return m_driveController;
    }

    /**
     * Get the second controller
     * 
     * @return The second controller
     */
    public XboxController getSecondController() {
        return m_operatorController;
    }

    /**
     * Set up the Shuffleboard
     */
    public void configureShuffleBoard() {
        ShuffleboardTab tab = Shuffleboard.getTab(Constants.DRIVER_READOUT_TAB_NAME);
        // tab.add("setPointUp", new InstantCommand(() -> m_ArmSubsystem.setpointUP()));
        // tab.add("setPointBack", new InstantCommand(() ->
        // m_ArmSubsystem.setpointBACK()));
        // tab.add("setPointForward", new InstantCommand(() ->
        // m_ArmSubsystem.setpointFORWARD()));
        // tab.add("setPointDown", new InstantCommand(() ->
        // m_ArmSubsystem.setpointDOWN()));
        if (!m_ChassisSubsystem.isTestRobot()) {
            tab.addNumber("Shoulder Setpoint", () -> m_ArmSubsystem.getShoulderSetpoint()).withPosition(3, 2);
            tab.addNumber("Elbow Setpoint", () -> m_ArmSubsystem.getElbowSetpoint()).withPosition(5, 2);
            tab.addNumber("Shoulder Angle", () -> m_ArmSubsystem.getShoulderJointDegrees()).withPosition(3, 0);
            tab.addNumber("Elbow Angle", () -> m_ArmSubsystem.getElbowJointDegrees()).withPosition(5, 0);
            tab.addString("CURRENT", () -> m_ArmSubsystem.getCurrentState()).withPosition(4, 1);
            tab.addString("UP", () -> m_ArmSubsystem.getUpState()).withPosition(4, 0);
            tab.addString("DOWN", () -> m_ArmSubsystem.getDownState()).withPosition(4, 2);
            tab.addString("FORWARD", () -> m_ArmSubsystem.getForwardState()).withPosition(5, 1);
            tab.addString("BACKWARD", () -> m_ArmSubsystem.getBackwardState()).withPosition(3, 1);
            tab.addNumber("Elbow Speed", () -> m_ArmSubsystem.getElbowSpeed());
            tab.addNumber("TOF Distance", () -> m_ManipulatorSubsystem.getDistance());
            tab.addNumber("Shoulder Velocity", () -> m_ArmSubsystem.getShoulderJointSpeed());
            tab.add(CameraServer.startAutomaticCapture("Camera", 0)).withSize(3, 3).withPosition(6, 0);
        }
        tab.add("Autonomous Mode", getAutonomousChooser().getModeChooser()).withSize(2, 1).withPosition(1, 0);
        // tab.add(m_drivetrainSubsystem.getField()).withSize(3, 2).withPosition(0, 1);
        tab.addBoolean("SLOW", () -> isSlow()).withPosition(2, 1);
        tab.addBoolean("TURBO", () -> isTurbo()).withPosition(1, 1);
        // tab.addBoolean("Auto", () ->
        // m_drivetrainSubsystem.getFollower().getCurrentTrajectory().isPresent());
    }

    /**
     * Setup all of the button controls for the robot
     */
    public void configureButtonBindings() {
        // Drivetrain
        new Trigger(m_driveController::getBackButton)
                .onTrue(new InstantCommand(m_DrivetrainSubsystem::zeroRotation, m_DrivetrainSubsystem));
        new Trigger(m_driveController::getRightBumper).debounce(0.1, DebounceType.kFalling)
                .onTrue(new InstantCommand(this::toggleSlow));
        new Trigger(m_driveController::getLeftBumper).debounce(0.1, DebounceType.kFalling)
                .onTrue(new InstantCommand(this::toggleTurbo));

        // Chassis
        new Trigger(m_operatorController::getStartButton)
                .onTrue(new InstantCommand(getChassisSubsystem()::setWantToggle,
                        getChassisSubsystem()));

        // Arm Setpoints
        if (!m_ChassisSubsystem.isTestRobot()) {
            new Trigger(m_operatorController::getYButton).debounce(0.5, DebounceType.kFalling).onTrue(
                    new InstantCommand(() -> m_ArmSubsystem.setpointUP()));
            new Trigger(m_operatorController::getXButton).debounce(0.5, DebounceType.kFalling).onTrue(
                    new InstantCommand(() -> m_ArmSubsystem.setpointBACK()));
            new Trigger(m_operatorController::getBButton).debounce(0.5, DebounceType.kFalling).onTrue(
                    new InstantCommand(() -> m_ArmSubsystem.setpointFORWARD()));
            new Trigger(m_operatorController::getAButton).debounce(0.5, DebounceType.kFalling).onTrue(
                    new InstantCommand(() -> m_ArmSubsystem.setpointDOWN()));
        }

        // Manipulator Commands
        // Intake
        if (!m_ChassisSubsystem.isTestRobot()) {
            new Trigger(m_operatorController::getLeftBumper)
                    .onTrue(new InstantCommand(m_ManipulatorSubsystem::intake, m_ManipulatorSubsystem));
            // Outtake
            new Trigger(m_operatorController::getRightBumper)
                    .onTrue(new InstantCommand(m_ManipulatorSubsystem::outtake, m_ManipulatorSubsystem));
            // StopTake
            new Trigger(m_driveController::getStartButton)
                    .onTrue(new InstantCommand(m_ManipulatorSubsystem::stoptake, m_ManipulatorSubsystem));
        }
        // new Trigger(m_operatorController::getXButton)
        // .onTrue(new InstantCommand(m_ManipulatorSubsystem::rotateLeft,
        // m_ManipulatorSubsystem));
        // new Trigger(m_operatorController::getXButton)
        // .onFalse(new InstantCommand(m_ManipulatorSubsystem::stopRotation,
        // m_ManipulatorSubsystem));
        // new Trigger(m_operatorController::getBButton)
        // .onTrue(new InstantCommand(m_ManipulatorSubsystem::rotateRight,
        // m_ManipulatorSubsystem));
        // new Trigger(m_operatorController::getBButton)
        // .onFalse(new InstantCommand(m_ManipulatorSubsystem::stopRotation,
        // m_ManipulatorSubsystem));

        // new Trigger(m_controller::getBButton)
        // .onTrue(new InstantCommand(() ->
        // m_ArmSubsystem.updateAllSetpoints(ArmSetpoints.MID_NODE)));
    }

    /**
     * Adjusts the input to remove the tolerance while retaining a smooth line with
     * tolerance as 0 and 100 as 100
     * 
     * @param value     The value to adjust
     * @param tolerance The amount of inner area to remove
     * @return The adjusted value
     */
    public static double deadband(double value, double tolerance) {
        if (Math.abs(value) < tolerance)
            return 0.0;

        return Math.copySign(value, (value - tolerance) / (1.0 - tolerance));
    }

    /**
     * Copy sign square
     * 
     * @param value Value to square
     * @return The copy sign square
     */
    public static double square(double value) {
        return Math.copySign(value * value, value);
    }

    /**
     * Get the adjusted Left Y axis of the main controller
     * 
     * @return The adjusted Left Y axis of the main controller
     */
    private double getForwardInput() {
        if (slow) {
            return -square(yLimiter.calculate(deadband(m_driveController.getLeftY(), 0.1))
                    * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND
                    * DrivetrainSubsystem.SPEED_MULTIPLIER * 0.2);
        } else if (turbo) {
            return -square(yLimiter.calculate(deadband(m_driveController.getLeftY(), 0.1))
                    * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND
                    * DrivetrainSubsystem.SPEED_MULTIPLIER * 1.5);
        } else {
            return -square(yLimiter.calculate(deadband(m_driveController.getLeftY(), 0.1))
                    * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND
                    * DrivetrainSubsystem.SPEED_MULTIPLIER);
        }
    }

    /**
     * Get the adjusted Left X axis of the main controller
     * 
     * @return The adjusted Left X axis of the main controller
     */
    private double getStrafeInput() {
        if (slow) {
            return -square(xLimiter.calculate(deadband(m_driveController.getLeftX(), 0.1))
                    * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND
                    * DrivetrainSubsystem.SPEED_MULTIPLIER * 0.2);
        } else if (turbo) {
            return -square(xLimiter.calculate(deadband(m_driveController.getLeftX(), 0.1))
                    * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND
                    * DrivetrainSubsystem.SPEED_MULTIPLIER * 1.4);
        } else {
            return -square(xLimiter.calculate(deadband(m_driveController.getLeftX(), 0.1))
                    * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND
                    * DrivetrainSubsystem.SPEED_MULTIPLIER);
        }
    }

    /**
     * Get the adjusted Right X axis of the main controller
     * 
     * @return The adjusted Right X axis of the main controller
     */
    private double getRotationInput() {
        if (slow) {
            return -square(deadband(m_driveController.getRightX(), 0.1))
                    * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * .2 * 0.33;
        } else {
            return -square(deadband(m_driveController.getRightX(), 0.1))
                    * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * .2;
        }
    }

    public boolean isSlow() {
        return slow;
    }

    public boolean isTurbo() {
        return turbo;
    }

    public void toggleSlow() {
        slow = !slow;
    }

    public void toggleTurbo() {
        turbo = !turbo;
    }

    /**
     * Accessor to the Autonomous Chooser
     * 
     * @return The Autonomous Chooser
     */
    public AutonomousChooser getAutonomousChooser() {
        return autonomousChooser;
    }

    /**
     * Accessor to the Arm Subsystem
     * 
     * @return The Arm Subsystem
     */
    public ArmSubsystem getArmSubsystem() {
        return m_ArmSubsystem;
    }

    /**
     * Accessor to the Manipulator Subsystem
     * 
     * @return The Manipulator Subsystem
     */
    public ManipulatorSubsystem getManipulatorSubsystem() {
        return m_ManipulatorSubsystem;
    }

    /**
     * Accessor to the Chassis Subsystem
     * 
     * @return The Chassis Subsystem
     */
    public ChassisSubsystem getChassisSubsystem() {
        return m_ChassisSubsystem;
    }

    /**
     * Accessor to the DriveTrain Subsystem
     * 
     * @return The DriveTrain Subsystem
     */
    public DrivetrainSubsystem getDrivetrain() {
        return m_DrivetrainSubsystem;
    }
}
