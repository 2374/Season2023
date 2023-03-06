package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
// import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmSetpoints;
// import frc.robot.Constants.ArmSetpoints;
import frc.robot.commands.*;
// import frc.robot.commands.helperCommands.ControlIntakeCommand;
import frc.robot.subsystems.*;
import frc.robot.subsystems.arm.ArmDefault;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.util.AutonomousChooser;
import frc.robot.util.AutonomousTrajectories;

public class RobotContainer {
    private final ChassisSubsystem m_ChassisSubsystem = new ChassisSubsystem();
    private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem(this);
    private final ArmSubsystem m_ArmSubsystem = new ArmSubsystem(this);
    private final ManipulatorSubsystem m_ManipulatorSubsystem = new ManipulatorSubsystem(this);
    // private final SimpleASubsystem m_ASubsystem = new SimpleASubsystem();

    private final AutonomousChooser autonomousChooser = new AutonomousChooser(
            new AutonomousTrajectories(DrivetrainSubsystem.TRAJECTORY_CONSTRAINTS));

    private final XboxController m_driveController = new XboxController(Constants.CONTROLLER_USB_PORT_DRIVER);
    private final XboxController m_operatorController = new XboxController(Constants.CONTROLLER_USB_PORT_OPERATOR);

    /**
     * The robot container. Need I say more?
     */
    public RobotContainer() {
        System.out.println("container created");

        resetDrive();
        resetArm();

        configureButtonBindings();
    }

    /**
     * Reset the default drive command
     */
    public void resetDrive() {
        m_drivetrainSubsystem.setDefaultCommand(
                new DefaultDriveCommand(m_drivetrainSubsystem, this::getForwardInput, this::getStrafeInput,
                        this::getRotationInput));
    }

    /**
     * Reset the default arm command
     */
    public void resetArm() {
        m_ArmSubsystem.reset();
        m_ArmSubsystem.setDefaultCommand(new ArmDefault(m_ArmSubsystem));
    }

    /**
     * Get the main controller
     * 
     * @return The main controller
     */
    public XboxController getController() {
        return m_driveController;
    }

    /**
     * Setup all of the button controls for the robot
     */
    public void configureButtonBindings() {
        new Trigger(m_driveController::getBackButton)
                .onTrue(new InstantCommand(m_drivetrainSubsystem::zeroRotation, m_drivetrainSubsystem));
        new Trigger(m_operatorController::getStartButton)
                .onTrue(new InstantCommand(getChassisSubsystem()::setWantToggle,
                        getChassisSubsystem()));
        new Trigger(m_operatorController::getLeftBumper)
                .onTrue(new InstantCommand(m_ArmSubsystem::incrementElbowSetPoint,
                m_ArmSubsystem));
        new Trigger(m_operatorController::getRightBumper)
                .onTrue(new InstantCommand(m_ArmSubsystem::decrementElbowSetPoint,
                m_ArmSubsystem));

        new Trigger(m_driveController::getLeftBumper)
                .onTrue(new InstantCommand(m_ArmSubsystem::incrementShoulderSetPoint,
                        m_ArmSubsystem));
        new Trigger(m_driveController::getRightBumper)
                .onTrue(new InstantCommand(m_ArmSubsystem::decrementShouldSetPoint,
                        m_ArmSubsystem));
                        

        new Trigger(m_driveController::getYButton).onTrue(
                new InstantCommand(() -> m_ArmSubsystem.updateAllSetpoints(ArmSetpoints.HILUNGE)));
        // new Trigger(m_driveController::getYButton).onTrue(
        //             new InstantCommand(() -> m_ArmSubsystem.midScoreRoutine()));
        new Trigger(m_driveController::getXButton).onTrue(
                new InstantCommand(() -> m_ArmSubsystem.updateAllSetpoints(ArmSetpoints.STOWED)));
        new Trigger(m_driveController::getBButton).onTrue(
                new InstantCommand(() -> m_ArmSubsystem.updateAllSetpoints(ArmSetpoints.ENGARDE)));
        new Trigger(m_driveController::getAButton).onTrue(
                new InstantCommand(() -> m_ArmSubsystem.updateAllSetpoints(ArmSetpoints.HIGARDE)));
       // new Trigger(m_operatorController::getPOV()).onTrue(new InstantCommand(m_ArmSubsystem::incrementShoulderSetPoint, m_ArmSubsystem));
        
        
        
        // new Trigger(m_controller::getYButton)
        // .onTrue(new InstantCommand(m_drivetrainSubsystem::printAngles,
        // m_drivetrainSubsystem));
        // Shoulder
        // new Trigger(m_controller::getXButton).onTrue(new
        // InstantCommand(m_ASubsystem::shoulderForward, m_ASubsystem));
        // new Trigger(m_controller::getXButton).onFalse(new
        // InstantCommand(m_ASubsystem::shoulderStop, m_ASubsystem));
        // new Trigger(m_controller::getAButton).onTrue(new
        // InstantCommand(m_ASubsystem::shoulderBack, m_ASubsystem));
        // new Trigger(m_controller::getAButton).onFalse(new
        // InstantCommand(m_ASubsystem::shoulderStop, m_ASubsystem));
        // Elbow
        // new Trigger(m_controller::getYButton).onTrue(new
        // InstantCommand(m_ASubsystem::elbowForward, m_ASubsystem));
        // new Trigger(m_controller::getYButton).onFalse(new
        // InstantCommand(m_ASubsystem::elbowStop, m_ASubsystem));
        // new Trigger(m_controller::getBButton).onTrue(new
        // InstantCommand(m_ASubsystem::elbowBack, m_ASubsystem));
        // new Trigger(m_controller::getBButton).onFalse(new
        // InstantCommand(m_ASubsystem::elbowStop, m_ASubsystem));
        
        // Manipulator Commands
        // Intake
        new Trigger(m_operatorController::getYButton).onTrue(new InstantCommand(m_ManipulatorSubsystem::intake, m_ManipulatorSubsystem));
        // Outtake
        new Trigger(m_operatorController::getAButton).onTrue(new InstantCommand(m_ManipulatorSubsystem::outtake, m_ManipulatorSubsystem));
        // StopTake
        new Trigger(m_operatorController::getBackButton).onTrue(new InstantCommand(m_ManipulatorSubsystem::stoptake, m_ManipulatorSubsystem));
        // Wrist
        new Trigger(m_operatorController::getXButton).onTrue(new InstantCommand(m_ManipulatorSubsystem::rotateLeft, m_ManipulatorSubsystem));
        new Trigger(m_operatorController::getXButton).onFalse(new InstantCommand(m_ManipulatorSubsystem::stopRotation, m_ManipulatorSubsystem));
        new Trigger(m_operatorController::getBButton).onTrue(new InstantCommand(m_ManipulatorSubsystem::rotateRight, m_ManipulatorSubsystem));
        new Trigger(m_operatorController::getBButton).onFalse(new InstantCommand(m_ManipulatorSubsystem::stopRotation, m_ManipulatorSubsystem));
        

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
    private static double deadband(double value, double tolerance) {
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
    private static double square(double value) {
        return Math.copySign(value * value, value);
    }

    /**
     * Get the adjusted Left Y axis of the main controller
     * 
     * @return The adjusted Left Y axis of the main controller
     */
    private double getForwardInput() {
        return -square(deadband(m_driveController.getLeftY(), 0.1)) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND
                * DrivetrainSubsystem.SPEED_MULTIPLIER;
    }

    /**
     * Get the adjusted Left X axis of the main controller
     * 
     * @return The adjusted Left X axis of the main controller
     */
    private double getStrafeInput() {
        return -square(deadband(m_driveController.getLeftX(), 0.1)) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND
                * DrivetrainSubsystem.SPEED_MULTIPLIER;
    }

    /**
     * Get the adjusted Right X axis of the main controller
     * 
     * @return The adjusted Right X axis of the main controller
     */
    private double getRotationInput() {
        return -square(deadband(m_driveController.getRightX(), 0.1))
                * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * DrivetrainSubsystem.SPEED_MULTIPLIER;
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
    // public ManipulatorSubsystem getManipulatorSubsystem() {
    // return m_ManipulatorSubsystem;
    // }

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
        return m_drivetrainSubsystem;
    }
}
