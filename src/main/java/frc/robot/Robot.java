package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.CharacterizeDrivetrainCommand;
import frc.robot.commands.automationCommands.AutoBalenceCommand;
import frc.robot.util.DriverReadout;

public class Robot extends TimedRobot {
    private final RobotContainer m_robotContainer = new RobotContainer();
    private Command m_autonomousCommand;

    @SuppressWarnings("unused")
    private final CharacterizeDrivetrainCommand characterizeCommand = new CharacterizeDrivetrainCommand(
            m_robotContainer.getDrivetrain());

    @SuppressWarnings("unused")
    private final DriverReadout driverReadout = new DriverReadout(m_robotContainer);

    @Override
    public void robotInit() {
        // drivetrain
        SmartDashboard.putData("Auto Balence", new AutoBalenceCommand(m_robotContainer.getDrivetrain()).withTimeout(5));

        // lights
        // SmartDashboard.putData("Signal Cone", new
        // InstantCommand(m_robotContainer.getLightsSubsystem()::turnOnYellow,
        // m_robotContainer.getLightsSubsystem()));
        // SmartDashboard.putData("Signal Cube", new
        // InstantCommand(m_robotContainer.getLightsSubsystem()::turnOnPurple,
        // m_robotContainer.getLightsSubsystem()));
        // SmartDashboard.putData("Signal Off", new
        // InstantCommand(m_robotContainer.getLightsSubsystem()::turnOff,
        // m_robotContainer.getLightsSubsystem()));

    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
        // lights
        // m_robotContainer.getLightsSubsystem().turnOff();
    }

    @Override
    public void disabledExit() {
        // robotContainer.getShooter().setHoodBrakeMode(false);
    }

    @Override
    public void teleopInit() {
        // if (!robotContainer.getClimber().isClimberZeroed()) {
        // new ZeroClimberCommand(robotContainer.getClimber()).schedule();
        // }
        // if (!robotContainer.getShooter().isHoodZeroed()) {
        // new ZeroHoodCommand(robotContainer.getShooter(), true).schedule();
        // }
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }

    }

    @Override
    public void testInit() {
        // new InstantCommand(robotContainer.getShooter()::disableFlywheel);
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousChooser().getCommand(m_robotContainer);
        // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }
}
