package frc.robot.commands.automationCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;

public class TopConeScoreCommand extends CommandBase {

    private final ArmSubsystem m_ArmSubsystem;
    private final ManipulatorSubsystem m_ManipulatorSubsystem;

    public TopConeScoreCommand(ArmSubsystem armSubsystem, ManipulatorSubsystem manipulatorSubsystem) {
        m_ArmSubsystem = armSubsystem;
        m_ManipulatorSubsystem = manipulatorSubsystem;

        addRequirements(m_ArmSubsystem);
        addRequirements(m_ManipulatorSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("Scoring Top");
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interupted) {

    }
}
