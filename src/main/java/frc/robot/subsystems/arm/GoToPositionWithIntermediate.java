// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import frc.robot.Constants.ArmSetpoints;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GoToPositionWithIntermediate extends SequentialCommandGroup {
    /** Creates a new GoToPositionWithIntermediate. */
    public GoToPositionWithIntermediate(ArmSubsystem arm, Setpoint setpoint) {

        Setpoint intermediateSetpoint = new Setpoint(0,ArmSetpoints.INTERMEDIATE_ELBOW_POSITION,
                setpoint.m_shoulderCone * 0.5, setpoint.wristCone,
                ArmSetpoints.INTERMEDIATE_ELBOW_POSITION, (setpoint.m_elbowCube) * 0.5, setpoint.wristCube);
        addCommands(
                new InstantCommand(() -> arm.updateAllSetpoints(intermediateSetpoint)),
                new WaitCommand(0.1),
                new WaitUntilCommand(() -> arm.bothJointsAtSetpoint()),
                new InstantCommand(() -> arm.updateAllSetpoints(setpoint)),
                new WaitCommand(0.1),
                new WaitUntilCommand(() -> arm.bothJointsAtSetpoint()));
    }
}
