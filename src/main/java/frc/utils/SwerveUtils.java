package frc.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;

public class SwerveUtils {

    public static Twist2d log(final Pose2d transform) {
        final double dtheta = transform.getRotation().getRadians();
        final double half_dtheta = 0.5 * dtheta;
        final double cos_minus_one = Math.cos(transform.getRotation().getRadians()) - 1.0;
        double halftheta_by_tan_of_halfdtheta;
        if (Math.abs(cos_minus_one) < 1E-9) {
            halftheta_by_tan_of_halfdtheta = 1.0 - 1.0 / 12.0 * dtheta * dtheta;
        } else {
            halftheta_by_tan_of_halfdtheta =
                    -(half_dtheta * Math.sin(transform.getRotation().getRadians())) / cos_minus_one;
        }
        final Translation2d translation_part =
                transform.getTranslation().rotateBy(new Rotation2d(halftheta_by_tan_of_halfdtheta, -half_dtheta));
        return new Twist2d(translation_part.getX(), translation_part.getY(), dtheta);
    }

    /**
     * Steps a value towards a target with a specified step size.
     *
     * @param _current The current or starting value. Can be positive or negative.
     * @param _target The target value the algorithm will step towards. Can be positive or negative.
     * @param _stepsize The maximum step size that can be taken.
     * @return The new value for {@code _current} after performing the specified step towards the
     *     specified target.
     */
    public static double StepTowards(double _current, double _target, double _stepsize) {
        if (Math.abs(_current - _target) <= _stepsize) {
            return _target;
        } else if (_target < _current) {
            return _current - _stepsize;
        } else {
            return _current + _stepsize;
        }
    }

    /**
     * Steps a value (angle) towards a target (angle) taking the shortest path with a specified step
     * size.
     *
     * @param _current The current or starting angle (in radians). Can lie outside the 0 to 2*PI
     *     range.
     * @param _target The target angle (in radians) the algorithm will step towards. Can lie outside
     *     the 0 to 2*PI range.
     * @param _stepsize The maximum step size that can be taken (in radians).
     * @return The new angle (in radians) for {@code _current} after performing the specified step
     *     towards the specified target. This value will always lie in the range 0 to 2*PI
     *     (exclusive).
     */
    public static double StepTowardsCircular(double _current, double _target, double _stepsize) {
        _current = WrapAngle(_current);
        _target = WrapAngle(_target);

        double stepDirection = Math.signum(_target - _current);
        double difference = Math.abs(_current - _target);

        if (difference <= _stepsize) {
            return _target;
        } else if (difference > Math.PI) { // does the system need to wrap over eventually?
            // handle the special case where you can reach the target in one step while also wrapping
            if (_current + 2 * Math.PI - _target < _stepsize || _target + 2 * Math.PI - _current < _stepsize) {
                return _target;
            } else {
                return WrapAngle(_current - stepDirection * _stepsize); // this will handle wrapping gracefully
            }

        } else {
            return _current + stepDirection * _stepsize;
        }
    }

    /**
     * Finds the (unsigned) minimum difference between two angles including calculating across 0.
     *
     * @param _angleA An angle (in radians).
     * @param _angleB An angle (in radians).
     * @return The (unsigned) minimum difference between the two angles (in radians).
     */
    public static double AngleDifference(double _angleA, double _angleB) {
        double difference = Math.abs(_angleA - _angleB);
        return difference > Math.PI ? (2 * Math.PI) - difference : difference;
    }

    /**
     * Wraps an angle until it lies within the range from 0 to 2*PI (exclusive).
     *
     * @param _angle The angle (in radians) to wrap. Can be positive or negative and can lie multiple
     *     wraps outside the output range.
     * @return An angle (in radians) from 0 and 2*PI (exclusive).
     */
    public static double WrapAngle(double _angle) {
        double twoPi = 2 * Math.PI;

        if (_angle == twoPi) { // Handle this case separately to avoid floating point errors with the floor
            // after the division in the case below
            return 0.0;
        } else if (_angle > twoPi) {
            return _angle - twoPi * Math.floor(_angle / twoPi);
        } else if (_angle < 0.0) {
            return _angle + twoPi * (Math.floor((-_angle) / twoPi) + 1);
        } else {
            return _angle;
        }
    }
}
