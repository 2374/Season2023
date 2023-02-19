package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
// import frc.robot.Constants.ArmSetpoints;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.util.AutonomousChooser;
import frc.robot.util.AutonomousTrajectories;

public class RobotContainer {
    private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
    private final ChassisSubsystem m_ChassisSubsystem = new ChassisSubsystem();
    // private final ArmSubsystem m_ArmSubsystem = new ArmSubsystem();
    private final ManipulatorSubsystem m_ManipulatorSubsystem = new ManipulatorSubsystem(this);
    private final SimpleASubsystem m_ASubsystem = new SimpleASubsystem();

    private final AutonomousChooser autonomousChooser = new AutonomousChooser(
            new AutonomousTrajectories(DrivetrainSubsystem.TRAJECTORY_CONSTRAINTS));

    private final XboxController m_controller = new XboxController(Constants.CONTROLLER_USB_PORT);

    public RobotContainer() {
        System.out.println("container created");

        resetDrive();

        configureButtonBindings();
    }

    public void resetDrive() {
        m_drivetrainSubsystem.setDefaultCommand(
                new DefaultDriveCommand(m_drivetrainSubsystem, this::getForwardInput, this::getStrafeInput,
                        this::getRotationInput));
    }

    public XboxController getController() {
        return m_controller;
    }

    // setup all of the button controls for the robot
    public void configureButtonBindings() {
        new Trigger(m_controller::getBackButton)
                .onTrue(new InstantCommand(m_drivetrainSubsystem::zeroRotation, m_drivetrainSubsystem));
        // new Trigger(m_controller::getYButton)
        // .onTrue(new InstantCommand(m_drivetrainSubsystem::printAngles,
        // m_drivetrainSubsystem));
        // upper
        // new Trigger(m_controller::getXButton).onTrue(new
        // InstantCommand(m_ASubsystem::upperForward, m_ASubsystem));
        // new Trigger(m_controller::getXButton).onFalse(new
        // InstantCommand(m_ASubsystem::upperStop, m_ASubsystem));
        // new Trigger(m_controller::getAButton).onTrue(new
        // InstantCommand(m_ASubsystem::upperBack, m_ASubsystem));
        // new Trigger(m_controller::getAButton).onFalse(new
        // InstantCommand(m_ASubsystem::upperStop, m_ASubsystem));
        // lower
        // new Trigger(m_controller::getYButton).onTrue(new
        // InstantCommand(m_ASubsystem::lowerForward, m_ASubsystem));
        // new Trigger(m_controller::getYButton).onFalse(new
        // InstantCommand(m_ASubsystem::lowerStop, m_ASubsystem));
        // new Trigger(m_controller::getBButton).onTrue(new
        // InstantCommand(m_ASubsystem::lowerBack, m_ASubsystem));
        // new Trigger(m_controller::getBButton).onFalse(new
        // InstantCommand(m_ASubsystem::lowerStop, m_ASubsystem));
        // wrist
        new Trigger(m_controller::getXButton)
                .onTrue(new InstantCommand(m_ManipulatorSubsystem::rotateLeft, m_ManipulatorSubsystem));
        new Trigger(m_controller::getXButton)
                .onFalse(new InstantCommand(m_ManipulatorSubsystem::stopRotation, m_ManipulatorSubsystem));
        new Trigger(m_controller::getAButton)
                .onTrue(new InstantCommand(m_ManipulatorSubsystem::rotateRight, m_ManipulatorSubsystem));
        new Trigger(m_controller::getAButton)
                .onFalse(new InstantCommand(m_ManipulatorSubsystem::stopRotation, m_ManipulatorSubsystem));
        // claw
        new Trigger(m_controller::getYButton)
                .onTrue(new InstantCommand(m_ManipulatorSubsystem::intake, m_ManipulatorSubsystem));
        new Trigger(m_controller::getYButton)
                .onFalse(new InstantCommand(m_ManipulatorSubsystem::stoptake, m_ManipulatorSubsystem));
        new Trigger(m_controller::getBButton)
                .onTrue(new InstantCommand(m_ManipulatorSubsystem::outtake, m_ManipulatorSubsystem));
        new Trigger(m_controller::getBButton)
                .onFalse(new InstantCommand(m_ManipulatorSubsystem::stoptake, m_ManipulatorSubsystem));

        // new Trigger(m_controller::getBButton)
        // .onTrue(new InstantCommand(() ->
        // m_ArmSubsystem.updateAllSetpoints(ArmSetpoints.MID_NODE)));
    }

    private static double deadband(double value, double tolerance) {
        if (Math.abs(value) < tolerance)
            return 0.0;

        return Math.copySign(value, (value - tolerance) / (1.0 - tolerance));
    }

    private static double square(double value) {
        return Math.copySign(value * value, value);
    }

    private double getForwardInput() {
        return -square(deadband(m_controller.getLeftY(), 0.1)) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND
                * DrivetrainSubsystem.SPEED_MULTIPLIER;
    }

    private double getStrafeInput() {
        return -square(deadband(m_controller.getLeftX(), 0.1)) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND
                * DrivetrainSubsystem.SPEED_MULTIPLIER;
    }

    private double getRotationInput() {
        return -square(deadband(m_controller.getRightX(), 0.1))
                * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * DrivetrainSubsystem.SPEED_MULTIPLIER;
    }

    public AutonomousChooser getAutonomousChooser() {
        return autonomousChooser;
    }

    // accessor to return the arm subsystem
    // public ArmSubsystem getArmSubsystem() {
    // return m_ArmSubsystem;
    // }

    // accessor to return the manipulator subsystem
    public ManipulatorSubsystem getManipulatorSubsystem() {
        return m_ManipulatorSubsystem;
    }

    // accessor to return the CANDle/light subsystem
    public ChassisSubsystem getChassisSubsystem() {
        return m_ChassisSubsystem;
    }

    // accessor to return the drive train subsystem
    public DrivetrainSubsystem getDrivetrain() {
        return m_drivetrainSubsystem;
    }
}
