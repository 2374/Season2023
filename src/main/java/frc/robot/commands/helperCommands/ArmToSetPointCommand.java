package frc.robot.commands.helperCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.Setpoint;

public class ArmToSetPointCommand extends CommandBase {
    private ArmSubsystem m_ArmSubsystem;
    private Setpoint setpoint;

    public ArmToSetPointCommand(ArmSubsystem armSubsystem, Setpoint setpoint) {
        m_ArmSubsystem = armSubsystem;
        this.setpoint = setpoint;
        addRequirements(m_ArmSubsystem);
    }

    @Override
    public void initialize() {
        m_ArmSubsystem.updateAllSetpoints(setpoint);
    }

    @Override
    public boolean isFinished() {
        return m_ArmSubsystem.bothJointsAtSetpoint();
    }
}
