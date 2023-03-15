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

public class MotorControlShoulder {


    WrapperedCANMotorCtrl motorCtrlLeft = new WrapperedCANMotorCtrl("Shoulder", Constants.SHOULDER_JOINT_LEFT_MOTOR_CAN_ID, CANMotorCtrlType.SPARK_MAX);
    WrapperedCANMotorCtrl motorCtrlRight = new WrapperedCANMotorCtrl("Shoulder Right", Constants.SHOULDER_JOINT_RIGHT_MOTOR_CAN_ID, CANMotorCtrlType.SPARK_MAX);
    private WPI_TalonFX m_shoulderLeftJoint = new WPI_TalonFX(Constants.SHOULDER_JOINT_LEFT_MOTOR_CAN_ID);
    private WPI_TalonFX m_shoulderRightJoint = new WPI_TalonFX(Constants.SHOULDER_JOINT_RIGHT_MOTOR_CAN_ID);

    //Feed Forward
    Calibration kF = new Calibration("Arm Shoulder kF", "V/degpersec", 0.12);
    Calibration kG = new Calibration("Arm Shoulder kG", "V/cos(deg)", 0.5);
    Calibration kS = new Calibration("Arm Shoulder kS", "V", 0.01);

    //Feedback
    Calibration kP = new Calibration("Arm Shoulder kP", "V/deg", 0.1);
    Calibration kI = new Calibration("Arm Shoulder kI", "V*sec/deg", 0.0);
    Calibration kD = new Calibration("Arm Shoulder kD", "V/degpersec", 0.00);

    // Gain schedule P 
    final double pDeazoneErrDeg = 1.375/2.0; //keep a very very tiny deadzone
    final double pDeadzonTransitionWidth = pDeazoneErrDeg/4.0;
    MapLookup2D closedLoopGainSchedule;

    PIDController m_pid = new PIDController(0, 0, 0);

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



    public MotorControlShoulder(){
        // make the right motor follow the left in ALL THINGS
        m_shoulderRightJoint.follow(m_shoulderLeftJoint);
        motorCtrlLeft.setBrakeMode(true);

        //Gain schedule P to have zero value in the deadzone
        // but get more powerful as error gets larger
        closedLoopGainSchedule = new MapLookup2D();
        closedLoopGainSchedule.insertNewPoint(-180, 1.0);
        closedLoopGainSchedule.insertNewPoint(-pDeazoneErrDeg - pDeadzonTransitionWidth, 1.0);
        closedLoopGainSchedule.insertNewPoint(-pDeazoneErrDeg, 0.0);
        closedLoopGainSchedule.insertNewPoint(pDeazoneErrDeg, 0.0);
        closedLoopGainSchedule.insertNewPoint(pDeazoneErrDeg + pDeadzonTransitionWidth, 1.0);
        closedLoopGainSchedule.insertNewPoint(180, 1.0);
    }

    public void setBrakeMode(boolean isBrakeMode){
        motorCtrlLeft.setBrakeMode(isBrakeMode);
    }

    public void setCmd(ArmAngularState testDesState){
        desAngleDeg = testDesState.shoulderAngleDeg;
        desAngVelDegPerSec = testDesState.shoulderAnglularVel;

        //Apply command limits
        // TODO determine and enable the min and max angles
        // if(desAngleDeg > Constants.ARM_SHOULDER_MAX_ANGLE_DEG){
        //     desAngleDeg = Constants.ARM_SHOULDER_MAX_ANGLE_DEG;
        //     desAngVelDegPerSec = 0.0;
        //     isAngleLimited = true;
        // } else if (desAngleDeg < Constants.ARM_SHOULDER_MIN_ANGLE_DEG){
        //     desAngleDeg = Constants.ARM__MIN_ANGLE_DEG;
        //     desAngVelDegPerSec = 0.0;
        //     isAngleLimited = true;
        // } else {
        //     isAngleLimited = false;
        // }

    }    

    public void update(ArmAngularState act_in, boolean enabled){

        actAngleDeg = act_in.shoulderAngleDeg;
        actAngVelDegPerSec = act_in.shoulderAnglularVel;

        var motorCmdV = 0.0;

        // update PID Controller Constants
        var absErr = Math.abs(actAngleDeg - desAngleDeg);
        m_pid.setPID(kP.get(), kI.get(), kD.get());

        // Calculate Feed-Forward
        cmdFeedForward = Math.signum(desAngVelDegPerSec) * kS.get() + 
                        Math.cos(Units.degreesToRadians(desAngleDeg)) * kG.get() + 
                        desAngVelDegPerSec * kF.get();

        // Update feedback command
        cmdFeedBack = m_pid.calculate(actAngleDeg, desAngleDeg) * closedLoopGainSchedule.lookupVal(absErr);

        motorCmdV = cmdFeedForward + cmdFeedBack;

        // Send total command to motor;
        motorCtrlLeft.setVoltageCmd(motorCmdV); 

        if(enabled){
            motorCtrlLeft.setVoltageCmd(-1.0 * (cmdFeedForward + cmdFeedBack)); 
        } else {
            motorCtrlLeft.setVoltageCmd(0.0);
        }
    }

}
