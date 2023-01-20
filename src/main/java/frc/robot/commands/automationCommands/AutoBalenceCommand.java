package frc.robot.commands.automationCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class AutoBalenceCommand extends CommandBase {

    private final RobotContainer m_robotContainer;

    public AutoBalenceCommand(RobotContainer robotContainer) {
        this.m_robotContainer = robotContainer;
        addRequirements(m_robotContainer.getDrivetrain());
    }

    @Override
    public void initialize() {
        m_robotContainer.getDrivetrain().getDefaultCommand().end(true);
    }

    @Override
    public void execute() {
        m_robotContainer.getDrivetrain().autoBalenceTick();
    }

    @Override
    public void end(boolean interupted) {
        m_robotContainer.resetDrive();
    }
}
