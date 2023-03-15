package frc.robot.subsystems.expArm;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.hardwareWrappers.MotorCtrl.WrapperedCANMotorCtrl;
import frc.hardwareWrappers.MotorCtrl.WrapperedCANMotorCtrl.CANMotorCtrlType;
import frc.lib.Calibration.Calibration;
import frc.lib.Signal.Annotations.Signal;
import frc.lib.Util.MapLookup2D;

public class MotorControlElbow {

    WrapperedCANMotorCtrl motorCtrlLeft = new WrapperedCANMotorCtrl("Elbow", Constants.ELBOW_JOINT_LEFT_MOTOR_CAN_ID, CANMotorCtrlType.TALON_FX);
    WrapperedCANMotorCtrl motorCtrlRight = new WrapperedCANMotorCtrl("Elbow Right", Constants.ELBOW_JOINT_RIGHT_MOTOR_CAN_ID, CANMotorCtrlType.TALON_FX);
    private WPI_TalonFX m_elbowLeftJoint = new WPI_TalonFX(Constants.ELBOW_JOINT_LEFT_MOTOR_CAN_ID);
    private WPI_TalonFX m_elbowRightJoint = new WPI_TalonFX(Constants.ELBOW_JOINT_RIGHT_MOTOR_CAN_ID);
    
    //Feed Forward
    Calibration kF = new Calibration("Arm Elbow kF", "V/degpersec", 0.11); 
    Calibration kG = new Calibration("Arm Elbow kG", "V/cos(deg)", 0.25);
    Calibration kS = new Calibration("Arm Elbow kS", "V", 0.1); 

    //Feedback
    Calibration kP = new Calibration("Arm Elbow kP", "V/deg", 0.30);
    Calibration kI = new Calibration("Arm Elbow kI", "V*sec/deg", 0.0);
    Calibration kD = new Calibration("Arm Elbow kD", "V/degpersec", 0.0);

    PIDController m_pid = new PIDController(0, 0, 0);

    // Gain schedule P 
    final double pDeazoneErrDeg =  2.1/2.0; //Due to mechanical bounce, Needs a deadzone of  7.67 deg or more
    final double pDeadzonTransitionWidth = pDeazoneErrDeg/4.0;
    MapLookup2D pGainSchedule;

    @Signal(units="V")
    double cmdFeedForward;
    @Signal(units="V")
    double cmdFeedBack;

    @Signal(units="deg")
    double desAngleDeg;
    @Signal(units="deg")
    double actAngleDeg;

    @Signal(units="degpersec")
    double desAngVelDegPerSec;
    @Signal(units="degpersec")
    double actAngVelDegPerSec;

    @Signal
    boolean isAngleLimited;

    public MotorControlElbow(){
        // make the right motor follow the left in ALL THINGS
        m_elbowRightJoint.follow(m_elbowLeftJoint);

        motorCtrlLeft.setBrakeMode(true);

        //Gain schedule P to have zero value in the deadzone
        // but get more powerful as error gets larger
        pGainSchedule = new MapLookup2D();
        pGainSchedule.insertNewPoint(-180, 1.0);
        pGainSchedule.insertNewPoint(-pDeazoneErrDeg - pDeadzonTransitionWidth, 1.0);
        pGainSchedule.insertNewPoint(-pDeazoneErrDeg, 0.0);
        pGainSchedule.insertNewPoint(pDeazoneErrDeg, 0.0);
        pGainSchedule.insertNewPoint(pDeazoneErrDeg + pDeadzonTransitionWidth, 1.0);
        pGainSchedule.insertNewPoint(180, 1.0);
    }

    public void setBrakeMode(boolean isBrakeMode){
        motorCtrlLeft.setBrakeMode(isBrakeMode);
    }

    public void setCmd(ArmAngularState in){
        desAngleDeg = in.elbowAngleDeg;
        desAngVelDegPerSec = in.elbowAngularVel;

        //Apply command limits
        // TODO determine max and min angles
        // if(desAngleDeg > Constants.ARM_ELBOW_MAX_ANGLE_DEG){
        //     desAngleDeg = Constants.ARM_ELBOW_MAX_ANGLE_DEG;
        //     desAngVelDegPerSec = 0.0;
        //     isAngleLimited = true;
        // } else if (desAngleDeg < Constants.ARM_ELBOW_MIN_ANGLE_DEG){
        //     desAngleDeg = Constants.ARM_ELBOW_MIN_ANGLE_DEG;
        //     desAngVelDegPerSec = 0.0;
        //     isAngleLimited = true;
        // } else {
        //     isAngleLimited = false;
        // }
    }

    public void update(ArmAngularState act_in, boolean enabled){

        actAngleDeg = act_in.elbowAngleDeg;
        actAngVelDegPerSec = act_in.elbowAngularVel;
        var actShoulderAngleDeg = act_in.shoulderAngleDeg;

        // update PID Controller Constants
        var absErr = Math.abs(actAngleDeg - desAngleDeg);
        m_pid.setPID(kP.get() * pGainSchedule.lookupVal(absErr), kI.get(), kD.get());

        //Calculate Feed-Forward
        cmdFeedForward = Math.signum(desAngVelDegPerSec) * kS.get() + 
                         Math.cos(Units.degreesToRadians(actAngleDeg + actShoulderAngleDeg)) * kG.get() + 
                         desAngVelDegPerSec * kF.get();


        cmdFeedBack = m_pid.calculate(actAngleDeg, desAngleDeg);

        if(enabled){
            motorCtrlLeft.setVoltageCmd(-1.0 * (cmdFeedForward + cmdFeedBack)); 
        } else {
            motorCtrlLeft.setVoltageCmd(0.0);
        }

    }
    
}
