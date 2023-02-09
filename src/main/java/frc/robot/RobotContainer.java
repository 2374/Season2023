package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.util.AutonomousChooser;
import frc.robot.util.AutonomousTrajectories;

public class RobotContainer {
    private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
    // private final LightsSubsystem m_lightsSubsystem = new LightsSubsystem();
    private final ArmSubsystem m_ArmSubsystem = new ArmSubsystem();
    private final ManipulatorSubsystem m_ManipulatorSubsystem = new ManipulatorSubsystem();

    private final AutonomousChooser autonomousChooser = new AutonomousChooser(
            new AutonomousTrajectories(DrivetrainSubsystem.TRAJECTORY_CONSTRAINTS));

    private final XboxController m_controller = new XboxController(Constants.CONTROLLER_PORT);

    public RobotContainer() {
        System.out.println("container created");
        // CommandScheduler.getInstance().registerSubsystem(m_drivetrainSubsystem);
        CommandScheduler.getInstance().registerSubsystem(m_ArmSubsystem);
        // CommandScheduler.getInstance().registerSubsystem(m_ManipulatorSubsystem);

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

    public void configureButtonBindings() {
        new Trigger(m_controller::getBackButton).onTrue(new InstantCommand(m_drivetrainSubsystem::zeroRotation));
        new Trigger(m_controller::getYButton).onTrue(new InstantCommand(m_drivetrainSubsystem::printAngles));
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

    public DrivetrainSubsystem getDrivetrain() {
        return m_drivetrainSubsystem;
    }

    public ArmSubsystem getArmSubsystem() {
        return m_ArmSubsystem;
    }

    public ManipulatorSubsystem getManipulatorSubsystem() {
        return m_ManipulatorSubsystem;
    }

    // public LightsSubsystem getLightsSubsystem() {
    // return m_lightsSubsystem;
    // }
}
