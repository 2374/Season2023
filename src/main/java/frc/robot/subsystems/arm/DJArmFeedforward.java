package frc.robot.subsystems.arm;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

public class DJArmFeedforward {
    private static final double g = 9.80665;

    private final JointConfig Joint_Lower;
    private final JointConfig Joint_Upper;

    public DJArmFeedforward(JointConfig Joint_Lower, JointConfig Joint_Upper) {
        this.Joint_Lower = Joint_Lower;
        this.Joint_Upper = Joint_Upper;
    }

    /** Calculates the joint voltages based on the joint positions (feedforward). */
    public Vector<N2> feedforward(Vector<N2> position) {
        return feedforward(position, VecBuilder.fill(0.0, 0.0), VecBuilder.fill(0.0, 0.0));
    }

    public Vector<N2> feedforward(Vector<N2> position, Vector<N2> velocity, Vector<N2> acceleration) {
        var torque = M(position)
                .times(acceleration)
                .plus(C(position, velocity).times(velocity))
                .plus(Tg(position));
        return VecBuilder.fill(
                Joint_Lower.motor.getVoltage(torque.get(0, 0), velocity.get(0, 0)),
                Joint_Upper.motor.getVoltage(torque.get(1, 0), velocity.get(1, 0)));
    }

    private Matrix<N2, N2> M(Vector<N2> position) {
        var M = new Matrix<>(N2.instance, N2.instance);
        M.set(
                0,
                0,
                Joint_Lower.mass * Math.pow(Joint_Lower.cgRadius, 2.0)
                        + Joint_Upper.mass * (Math.pow(Joint_Lower.length, 2.0) + Math.pow(Joint_Upper.cgRadius, 2.0))
                        + Joint_Lower.moi
                        + Joint_Upper.moi
                        + 2
                                * Joint_Upper.mass
                                * Joint_Lower.length
                                * Joint_Upper.cgRadius
                                * Math.cos(position.get(1, 0)));
        M.set(
                1,
                0,
                Joint_Upper.mass * Math.pow(Joint_Upper.cgRadius, 2.0)
                        + Joint_Upper.moi
                        + Joint_Upper.mass * Joint_Lower.length * Joint_Upper.cgRadius * Math.cos(position.get(1, 0)));
        M.set(
                0,
                1,
                Joint_Upper.mass * Math.pow(Joint_Upper.cgRadius, 2.0)
                        + Joint_Upper.moi
                        + Joint_Upper.mass * Joint_Lower.length * Joint_Upper.cgRadius * Math.cos(position.get(1, 0)));
        M.set(1, 1, Joint_Upper.mass * Math.pow(Joint_Upper.cgRadius, 2.0) + Joint_Upper.moi);
        return M;
    }

    private Matrix<N2, N2> C(Vector<N2> position, Vector<N2> velocity) {
        var C = new Matrix<>(N2.instance, N2.instance);
        C.set(
                0,
                0,
                -Joint_Upper.mass
                        * Joint_Lower.length
                        * Joint_Upper.cgRadius
                        * Math.sin(position.get(1, 0))
                        * velocity.get(1, 0));
        C.set(
                1,
                0,
                Joint_Upper.mass
                        * Joint_Lower.length
                        * Joint_Upper.cgRadius
                        * Math.sin(position.get(1, 0))
                        * velocity.get(0, 0));
        C.set(
                0,
                1,
                -Joint_Upper.mass
                        * Joint_Lower.length
                        * Joint_Upper.cgRadius
                        * Math.sin(position.get(1, 0))
                        * (velocity.get(0, 0) + velocity.get(1, 0)));
        return C;
    }

    private Matrix<N2, N1> Tg(Vector<N2> position) {
        var Tg = new Matrix<>(N2.instance, N1.instance);
        Tg.set(
                0,
                0,
                (Joint_Lower.mass * Joint_Lower.cgRadius + Joint_Upper.mass * Joint_Lower.length)
                        * g
                        * Math.cos(position.get(0, 0))
                        + Joint_Upper.mass
                                * Joint_Upper.cgRadius
                                * g
                                * Math.cos(position.get(0, 0) + position.get(1, 0)));
        Tg.set(
                1,
                0,
                Joint_Upper.mass * Joint_Upper.cgRadius * g * Math.cos(position.get(0, 0) + position.get(1, 0)));
        return Tg;
    }

}