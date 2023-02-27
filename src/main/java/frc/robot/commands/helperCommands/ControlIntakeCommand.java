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
    public void initialize() {
        System.out.println(cone);
        if (in) {
            m_ManipulatorSubsystem.intake();
        } else {
            m_ManipulatorSubsystem.outtake();
        }
    }

    @Override
    public boolean isFinished() {
        if (in) {
            if ((cone && m_ManipulatorSubsystem.getDistance() < 30)
                    || (!cone && m_ManipulatorSubsystem.getDistance() < 100)) {
                return true;
            }
        } else {
            if (m_ManipulatorSubsystem.getDistance() > 350) {
                return true;
            }
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Stop");
        m_ManipulatorSubsystem.stoptake();
    }
}
