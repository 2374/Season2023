package frc.robot.util;

import java.util.Objects;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUsageId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class AdvancedTrapezoidProfile extends TrapezoidProfile {

    private int m_direction;

    private Constraints m_constraints;
    private State m_initial;
    private State m_goal;

    private double m_endAccel;
    private double m_endFullSpeed;
    private double m_endDeccel;

    public static class Constraints {
        public final double maxVelocity;

        public final double maxAcceleration;

        public final double maxDecceleration;

        /**
         * Construct constraints for a TrapezoidProfile.
         *
         * @param maxVelocity     maximum velocity
         * @param maxAcceleration maximum acceleration
         */
        public Constraints(double maxVelocity, double maxAcceleration, double maxDecceleration) {
            this.maxVelocity = maxVelocity;
            this.maxAcceleration = maxAcceleration;
            this.maxDecceleration = maxDecceleration;
            MathSharedStore.reportUsage(MathUsageId.kTrajectory_TrapezoidProfile, 1);
        }
    }

    public static class State {
        public double position;

        public double velocity;

        public State() {
        }

        public State(double position, double velocity) {
            this.position = position;
            this.velocity = velocity;
        }

        @Override
        public boolean equals(Object other) {
            if (other instanceof State) {
                State rhs = (State) other;
                return this.position == rhs.position && this.velocity == rhs.velocity;
            } else {
                return false;
            }
        }

        @Override
        public int hashCode() {
            return Objects.hash(position, velocity);
        }
    }

    /**
     * Construct a TrapezoidProfile.
     *
     * @param constraints The constraints on the profile, like maximum velocity.
     * @param goal        The desired state when the profile is complete.
     * @param initial     The initial state (usually the current state).
     */
    public AdvancedTrapezoidProfile(Constraints constraints, State goal, State initial) {
        m_direction = shouldFlipAcceleration(initial, goal) ? -1 : 1;
        m_constraints = constraints;
        m_initial = direct(initial);
        m_goal = direct(goal);

        if (m_initial.velocity > m_constraints.maxVelocity) {
            m_initial.velocity = m_constraints.maxVelocity;
        }

        // Deal with a possibly truncated motion profile (with nonzero initial or
        // final velocity) by calculating the parameters as if the profile began and
        // ended at zero velocity
        double cutoffBegin = m_initial.velocity / m_constraints.maxAcceleration;
        double cutoffDistBegin = cutoffBegin * cutoffBegin * m_constraints.maxAcceleration / 2.0;

        double cutoffEnd = m_goal.velocity / m_constraints.maxDecceleration;
        double cutoffDistEnd = cutoffEnd * cutoffEnd * m_constraints.maxDecceleration / 2.0;

        // Now we can calculate the parameters as if it was a full trapezoid instead
        // of a truncated one

        double fullTrapezoidDist = cutoffDistBegin + (m_goal.position - m_initial.position) + cutoffDistEnd;
        double accelerationTime = m_constraints.maxVelocity / m_constraints.maxAcceleration;
        double deccelerationTime = m_constraints.maxVelocity / m_constraints.maxDecceleration;

        double fullSpeedDist = fullTrapezoidDist
                - accelerationTime * accelerationTime * m_constraints.maxAcceleration / 2
                - deccelerationTime * deccelerationTime * m_constraints.maxDecceleration / 2;

        // Handle the case where the profile never reaches full speed

        // TODO NEEDS TO BE FIXED
        if (fullSpeedDist < 0) {
            accelerationTime = Math.sqrt(fullTrapezoidDist / m_constraints.maxAcceleration);
            fullSpeedDist = 0;
        }

        m_endAccel = accelerationTime - cutoffBegin;
        m_endFullSpeed = m_endAccel + fullSpeedDist / m_constraints.maxVelocity;
        m_endDeccel = m_endFullSpeed + accelerationTime - cutoffEnd;
    }

    private static boolean shouldFlipAcceleration(State initial, State goal) {
        return initial.position > goal.position;
    }

    // Flip the sign of the velocity and position if the profile is inverted
    private State direct(State in) {
        State result = new State(in.position, in.velocity);
        result.position = result.position * m_direction;
        result.velocity = result.velocity * m_direction;
        return result;
    }
}
