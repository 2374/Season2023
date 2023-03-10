package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.automationCommands.AlignWithAprilTagCommand;
import frc.robot.commands.automationCommands.AutoBalenceCommand;
import frc.robot.util.DriverReadout;

public class Robot extends TimedRobot {
    private final RobotContainer m_robotContainer = new RobotContainer();

    private Command m_autonomousCommand;

    // @SuppressWarnings("unused")
    // private final CharacterizeDrivetrainCommand characterizeCommand = new
    // CharacterizeDrivetrainCommand(
    // m_robotContainer.getDrivetrain());

    @SuppressWarnings("unused")
    private final DriverReadout driverReadout = new DriverReadout(m_robotContainer);

    @Override
    public void robotInit() {
        // drivetrain
        SmartDashboard.putData("Auto Balence", new AutoBalenceCommand(m_robotContainer.getDrivetrain()).withTimeout(5));
        SmartDashboard.putNumber("Apriltag Number", 1);
        SmartDashboard.putData("Align With Apriltag", new AlignWithAprilTagCommand(m_robotContainer.getDrivetrain(),
                (int) SmartDashboard.getNumber("Apriltag Number", 1)));

        // game piece determination
        SmartDashboard.putData("Signal Cone", new InstantCommand(m_robotContainer.getChassisSubsystem()::setWantACone,
                m_robotContainer.getChassisSubsystem()));
        SmartDashboard.putData("Signal Cube", new InstantCommand(m_robotContainer.getChassisSubsystem()::setWantACube,
                m_robotContainer.getChassisSubsystem()));
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
        // lights
        m_robotContainer.getArmSubsystem().reset();
        m_robotContainer.getChassisSubsystem().setWantNothing();
    }

    @Override
    public void disabledExit() {
        m_robotContainer.getArmSubsystem().reset();
        m_robotContainer.getChassisSubsystem().setWantACone();
        m_robotContainer.getManipulatorSubsystem().resetEncoder();
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
