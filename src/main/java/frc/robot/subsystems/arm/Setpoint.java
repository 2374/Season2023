// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

/** Add your docs here. */
public class Setpoint {
    public double m_elbowCone;
    public double m_shoulderCone;
    public boolean wristCone;
    public double m_elbowCube;
    public double m_shoulderCube;
    public boolean wristCube;

    /**
     * 
     * @param m_elbowCone
     * @param m_shoulderCone
     * @param wristCone
     * @param m_elbowCube
     * @param m_shoulderCube
     * @param wristCube
     */
    public Setpoint(double m_elbowCone, double m_shoulderCone, boolean wristCone, double m_elbowCube,
            double m_shoulderCube,
            boolean wristCube) {
        this.m_elbowCone = m_elbowCone;
        this.m_shoulderCone = m_shoulderCone;
        this.wristCone = wristCone;
        this.m_elbowCube = m_elbowCube;
        this.m_shoulderCube = m_shoulderCube;
        this.wristCube = wristCube;
    }
}
