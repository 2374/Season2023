package frc.robot.subsystems.expArm;


public class ArmAngularState {
    //used for angles of arms
    public double shoulderAngleDeg;
    public double shoulderAnglularVel;
    public double elbowAngleDeg;
    public double elbowAngularVel;

    public ArmAngularState(double shoudderAngleDeg, double shoulderAnglularVel, double elbowAngleDeg, double elbowAngularVel){
        this.shoulderAngleDeg = shoudderAngleDeg;
        this.elbowAngleDeg = elbowAngleDeg;
        this.shoulderAnglularVel = shoulderAnglularVel;
        this.elbowAngularVel = elbowAngularVel;    
    }


    public ArmAngularState(double shoulderAngleDeg, double elbowAngleDeg){
        this.shoulderAngleDeg = shoulderAngleDeg;
        this.elbowAngleDeg = elbowAngleDeg;
        this.shoulderAnglularVel = 0;
        this.elbowAngularVel = 0;    
    }

    public ArmAngularState(){
        this.shoulderAngleDeg = 0;
        this.elbowAngleDeg = 0;
        this.shoulderAnglularVel = 0;
        this.elbowAngularVel = 0;    
    }



}