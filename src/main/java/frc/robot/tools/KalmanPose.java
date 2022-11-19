package frc.robot.tools;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;

/**
 * A class used to set up the Kalman Filter that will be used to get even better
 * readings of the robot postion on the field.
 */
public class KalmanPose {
    // TODO: Must check these values with Dr. E
    /** System Matrix */
    // ⎡ 1, 𝜟t ⎤
    // ⎣ 0, 1 ⎦
    private final Matrix<N2, N2> a = new Matrix<>(new SimpleMatrix(new double[][] { { 0.0, 1.0 }, { 0.0, 0.0 } }));

    /** Input Matrix */
    // ⎡ 𝜟t²/2 ⎤
    // ⎣ 𝜟t ⎦
    private final Matrix<N2, N1> b = new Matrix<>(new SimpleMatrix(new double[][] { { 0.0002 }, { 0.02 } }));

    /** Output Matrix */
    // ⎡ 1 ⎤
    // ⎣ 0 ⎦
    private final Matrix<N1, N2> c = new Matrix<>(new SimpleMatrix(new double[][] { { 1.0, 0.0 } }));

    /** FeedForward Matrix */
    // ⎡ 0 ⎤
    // ⎣ 0 ⎦
    private final Matrix<N1, N1> d = new Matrix<>(new SimpleMatrix(new double[][] { { 0.0, 0.0 } }));

    /** "Plant" Matrix */
    private final LinearSystem<N2, N1, N1> linearSystem = new LinearSystem<N2, N1, N1>(a, b, c, d);

    private KalmanFilter<N2, N1, N1> skObserver = new KalmanFilter<N2, N1, N1>(
            Nat.N2(),
            Nat.N1(),
            linearSystem,
            VecBuilder.fill(1.0, 1.0), // How accurate we think our model is
            VecBuilder.fill(3.0), // How accurate we think our encoder
            0.020); // Time increments of 20ms

    // private KalmanFilter<N2, N1, N1> observer =
    // new KalmanFilter<N2, N1, N1>(
    // Nat.N2(),
    // Nat.N1(),
    // linearSystem,
    // VecBuilder.fill(1.0, 1.0), // How accurate we think our model is
    // VecBuilder.fill(3.0), // How accurate we think our encoder
    // 0.20);

    private Matrix<N1, N1> u = new Matrix<N1, N1>(new SimpleMatrix(new double[][] { { 0 } }));
    private Matrix<N1, N1> z = new Matrix<N1, N1>(new SimpleMatrix(new double[][] { { 0 } }));

    /**
     * Creates a new KalmanPose class which is used to update the Kalman filter in
     * one axis
     */
    public KalmanPose() {
    }

    /**
     * Must be called in the periodic function. This will predict and correct the
     * state using the Kalman Filters
     * 
     * @param uDouble The input value
     * @param zDouble The measurement value
     */
    public void periodic(double uDouble, double zDouble) {
        predict(uDouble);
        correct(zDouble);

        // skObserver.correct(u, z);

        // observer.predict(u, 0.020);
        // observer.correct(u, z);
    }

    /**
     * Must be called in the periodic function in order to properly predict the new
     * position and velocity. Can call the periodic function instead if the
     * measurement value is known
     * 
     * @param uDouble
     */
    public void predict(double uDouble) {
        u.set(0, 0, uDouble);
        skObserver.predict(u, 0.020);
    }

    /**
     * Called whenever a measurement value has been received. Will be used to correct
     * the current prediction value towards the measurement value.
     * @param zDouble
     */
    public void correct(double zDouble) {
        z.set(0, 0, zDouble);
        skObserver.correct(u, z, 0.020);
    }

    /**
     * Gets the position as calculated by {@link SKKalmanFilter}
     * 
     * @return The calculated state of the system
     */
    public double getPosition() {
        return skObserver.getXhat().get(0, 0);
    }

    /**
     * Gets the velocity as calculated by {@link SKKalmanFilter}
     * 
     * @return The calculated state of the system
     */
    public double getVelocity() {
        return skObserver.getXhat().get(1, 0);
    }

    // /**
    // * Gets the state as calculated by the WPILib {@link KalmanFilter}
    // * @return The calculated state of the system
    // */
    // public double getStateWPI()
    // {
    // return observer.getXhat().get(0, 0);
    // }

    /**
     * Sets the value of position of the robot
     * 
     * @param x The new position of the robot in meters
     */
    public void setPosition(double x) {
        skObserver.setXhat(0, x);
        // observer.setXhat(0, x);
    }

    /**
     * Sets the value of position of the robot
     * 
     * @param x The new position of the robot in meters
     */
    public void setInitialVel(double x) {
        skObserver.setXhat(1, x);
        // observer.setXhat(0, x);
    }
}