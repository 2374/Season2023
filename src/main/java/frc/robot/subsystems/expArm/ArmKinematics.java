package frc.robot.subsystems.expArm;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;

public class ArmKinematics {

    public static ArmEndEffectorState forward(ArmAngularState in) {
        // Calcualte current, previous and next angles for each joint
        var shoulderAngleCur = in.shoulderAngleDeg;
        var shoulderAngleNext = shoulderAngleCur + in.shoulderAnglularVel * 0.02;
        var shoulderAnglePrev = shoulderAngleCur - in.shoulderAnglularVel * 0.02;

        var elbowAngleCur = in.elbowAngleDeg;
        var elbowAngleNext = elbowAngleCur + in.elbowAngularVel * 0.02;
        var elbowAnglePrev = elbowAngleCur - in.elbowAngularVel * 0.02;

        // Convert current, previous, and next angles into x/y positions
        var cur = forward_internal(shoulderAngleCur, elbowAngleCur);
        var prev = forward_internal(shoulderAnglePrev, elbowAnglePrev);
        var next = forward_internal(shoulderAngleNext, elbowAngleNext);

        // calcualte velocities assuming constant velocity motion from previous to next
        // point.
        var xVel = (next.getFirst() - prev.getFirst()) / 0.04;
        var yVel = (next.getSecond() - prev.getSecond()) / 0.04;

        // Return a complete end effector state object.
        return new ArmEndEffectorState(cur.getFirst(), cur.getSecond(), xVel, yVel, isReflex(elbowAngleCur));

    }

    private static boolean isReflex(double elbowAngleDeg){
        return (elbowAngleDeg < 0);
    }

    private static Pair<Double, Double> forward_internal(double shoulderAngleDeg, double elbowAngleDeg) {

        var shoulderRad = Units.degreesToRadians(shoulderAngleDeg);
        var elbowRad = Units.degreesToRadians(elbowAngleDeg);

        double armX = Constants.ARM_BICEP_LENGTH * Math.cos(shoulderRad) +
                Constants.ARM_FOREARM_LENGTH * Math.cos(shoulderRad + elbowRad);

        double armY = Constants.ARM_SHOULDER_MOUNT_HEIGHT +
                Constants.ARM_BICEP_LENGTH * Math.sin(shoulderRad) +
                Constants.ARM_FOREARM_LENGTH * Math.sin(shoulderRad + elbowRad);

        return Pair.of(armX, armY);
    }

    public static ArmAngularState inverse(ArmEndEffectorState in) {

        //Calculate current, previous, and next x/y positions
        var curX = in.x;
        var curY = in.y;
        var curReflex = in.isReflex;

        var nextX = in.x + in.xvel * 0.02;
        var nextY = in.y + in.yvel * 0.02;

        var prevX = in.x - in.xvel * 0.02;
        var prevY = in.y - in.yvel * 0.02;

        //Run all three through kinematics
        var cur = inverse_internal(curX, curY, curReflex);
        var next = inverse_internal(nextX, nextY, curReflex);
        var prev = inverse_internal(prevX, prevY, curReflex);

        //Calcualte angles and angular velocities
        var shoulderAngleDeg = Units.radiansToDegrees(cur.getFirst());
        var elbowkAngleDeg = Units.radiansToDegrees(cur.getSecond());
        var shoulderAnglularVel = 0.0;
        var elbowAngularVel = 0.0;
        if(next != null && prev != null){
            shoulderAnglularVel = Units.radiansToDegrees(next.getFirst() - prev.getFirst())/0.04;
            elbowAngularVel = Units.radiansToDegrees(next.getSecond() - prev.getSecond())/0.04;
        }
        
        return new ArmAngularState(shoulderAngleDeg, shoulderAnglularVel, elbowkAngleDeg, elbowAngularVel);   
    }

    private static Pair<Double, Double> inverse_internal(double x, double y, boolean isReflex) {
        // from
        // https://robotacademy.net.au/lesson/inverse-kinematics-for-a-2-joint-robot-arm-using-geometry/

        // Reset the coordinate system back down to have the shoulder pivot at the origin
        // just makes the subsequent math easier.
        y = y - Constants.ARM_SHOULDER_MOUNT_HEIGHT;

        double elbowDenom = 2 * Constants.ARM_BICEP_LENGTH * Constants.ARM_FOREARM_LENGTH;
        double elbowNumerator = Math.pow(x, 2) +
                Math.pow(y, 2) -
                Math.pow(Constants.ARM_BICEP_LENGTH, 2) -
                Math.pow(Constants.ARM_FOREARM_LENGTH, 2);

        double elbowAngleRad = Math.acos(elbowNumerator / elbowDenom) * (isReflex ? -1.0 : 1.0);

        double shoulderTerm1 = Math.atan2(y, x);

        double shoulderTerm2Num = Constants.ARM_FOREARM_LENGTH * Math.sin(elbowAngleRad * (isReflex ? -1.0 : 1.0));
        double shoulderTerm2Denom = Constants.ARM_BICEP_LENGTH
                + Constants.ARM_FOREARM_LENGTH * Math.cos(elbowAngleRad * (isReflex ? -1.0 : 1.0));

        double shoulderTerm2 = Math.atan2(shoulderTerm2Num , shoulderTerm2Denom) * (isReflex ? 1.0 : -1.0);

        double shoulderAngleRad = shoulderTerm1 + shoulderTerm2;

        if(!Double.isFinite(shoulderAngleRad) || !Double.isFinite(elbowAngleRad)){
            return null; //Unreachable position
        } else {
            return Pair.of((Double) shoulderAngleRad, (Double) elbowAngleRad);
        }

    }

}
