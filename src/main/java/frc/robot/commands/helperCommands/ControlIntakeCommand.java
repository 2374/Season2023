package frc.robot.commands.helperCommands;

// import com.swervedrivespecialties.swervelib.rev.NeoSteerControllerFactoryBuilder.ControllerImplementation;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ManipulatorSubsystem;

public class ControlIntakeCommand extends CommandBase {
    private final ManipulatorSubsystem m_ManipulatorSubsystem;
    private final boolean in;
    private final boolean cone;

    public ControlIntakeCommand(ManipulatorSubsystem manipulatorSubsystem, boolean in, boolean cone) {
        m_ManipulatorSubsystem = manipulatorSubsystem;
        this.in = in;
        this.cone = cone;
        addRequirements(m_ManipulatorSubsystem);
    }

    @Override
    public void execute() {
        if (in) {
            if (cone && m_ManipulatorSubsystem.getDistance() > 30) {
                m_ManipulatorSubsystem.intake();
            } else if (!cone && m_ManipulatorSubsystem.getDistance() > 60) {
                m_ManipulatorSubsystem.intake();
            } else {
                end(true);
            }
        } else {
            if (m_ManipulatorSubsystem.getDistance() < 150) {
                m_ManipulatorSubsystem.outtake();
            } else {
                end(true);
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_ManipulatorSubsystem.stop();
    }
}
