// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.IMUWraps.SK_ADIS16470_IMU;
import frc.robot.tools.KalmanPose;

public class IMUSubsystem extends SubsystemBase {
    private SK_ADIS16470_IMU imu = new SK_ADIS16470_IMU();

    private KalmanPose kalmanX = new KalmanPose();
    private KalmanPose kalmanY = new KalmanPose();

    double alpha = 0.0;
    double beta = 0.0;
    double gamma = 0.0;

    double xAccel = 0.0;
    double yAccel = 0.0;
    double zAccel = 0.0;

    long counter = 0;

    /** Creates a new ExampleSubsystem. */
    public IMUSubsystem() {
    }

    @Override
    public void periodic() {
        imu.addDataPoint();

        alpha = Math.toRadians(imu.getYaw());
        gamma = Math.toRadians(imu.getPitch());
        beta = Math.toRadians(imu.getRoll());

        xAccel = -imu.getAccelX();
        yAccel = -imu.getAccelY();
        zAccel = -imu.getAccelZ();
        
        double[] globalAccel = getGlobalAccel();
        
        kalmanX.predict(globalAccel[0]);
        kalmanY.predict(globalAccel[1]);

        // This method will be called once per scheduler run
        
        SmartDashboard.putNumber("X Position", kalmanX.getPosition());
        SmartDashboard.putNumber("X Velocity", kalmanX.getVelocity());
        SmartDashboard.putNumber("Y Position", kalmanY.getPosition());
        SmartDashboard.putNumber("Y Velocity", kalmanY.getVelocity());

        SmartDashboard.putNumber("Global X", globalAccel[0]);
        SmartDashboard.putNumber("Global Y", globalAccel[1]);
        SmartDashboard.putNumber("Global Z", globalAccel[2]);

        SmartDashboard.putNumber("Delta Z Accel", globalAccel[2] - zAccel);
        SmartDashboard.putNumber("Global Accel Residual", 9.81-(Math.sqrt((globalAccel[0]*globalAccel[0])+(globalAccel[1]*globalAccel[1])+(globalAccel[2]*globalAccel[2]))));

        double preRotated = Math.sqrt(Math.pow(xAccel, 2) + Math.pow(yAccel, 2) + Math.pow(zAccel, 2));
        double postRotated = Math.sqrt(Math.pow(globalAccel[0], 2) + Math.pow(globalAccel[1], 2) + Math.pow(globalAccel[2], 2));

        SmartDashboard.putNumber("Pre-Rotated Magnitude", preRotated);
        SmartDashboard.putNumber("Rotated Magnitude", postRotated);
        SmartDashboard.putNumber("Delta Magnitude", postRotated - preRotated);

        counter++;
        if (counter % 50 == 0)
        {
            SmartDashboard.putNumber("Counter", counter);
        }
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    /**
     * Calculates the global acceleration of the imu relative to the field and
     * given yaw angle
     * 
     * @return An array of global acceleration with the following format:
     *         {X Accel, Y Accel, Z Accel}
     */
    public double[] getGlobalAccel() {

        SmartDashboard.putNumber("Pitch", Math.toDegrees(beta));
        SmartDashboard.putNumber("Roll", Math.toDegrees(gamma));
        SmartDashboard.putNumber("Yaw", Math.toDegrees(alpha));

        SmartDashboard.putNumber("X Accel", xAccel);
        SmartDashboard.putNumber("Y Accel", yAccel);
        SmartDashboard.putNumber("Z Accel", zAccel);

        alpha = -alpha;
        // beta = -beta;
        // gamma = -gamma;

        // alpha = Math.toRadians(70);
        // beta = Math.toRadians(0);
        // gamma = Math.toRadians(90);

        Matrix<N3, N3> yawMatrix = new Matrix<>(new SimpleMatrix(new double[][]
                { { Math.cos(alpha), -Math.sin(alpha), 0 },
                { Math.sin(alpha), Math.cos(alpha), 0 },
                { 0, 0, 1 } }));

        Matrix<N3, N3> pitchMatrix = new Matrix<>(new SimpleMatrix(new double[][]
                { { Math.cos(beta), 0, Math.sin(beta) },
                { 0, 1, 0 },
                { -Math.sin(beta), 0, Math.cos(beta) } }));

        Matrix<N3, N3> rollMatrix = new Matrix<>(new SimpleMatrix(new double[][]
                { { 1, 0, 0 },
                { 0, Math.cos(gamma), -Math.sin(gamma) },
                { 0, Math.sin(gamma), Math.cos(gamma) } }));

        Matrix<N3, N1> accelMatrix = new Matrix<>(new SimpleMatrix(new double[][]
                { { xAccel },
                { yAccel },
                { zAccel } }));

        Matrix<N3, N3> combined = yawMatrix.times(pitchMatrix).times(rollMatrix);
        // Matrix<N3, N3> inverse = combined.inv();

        SmartDashboard.putNumber("Det of R", combined.det());

        SmartDashboard.putNumber("Single Rotated X", rollMatrix.times(accelMatrix).get(0, 0));
        SmartDashboard.putNumber("Single Rotated Y", rollMatrix.times(accelMatrix).get(1, 0));

        Matrix<N3, N1> result = combined.times(accelMatrix);

        alpha = -alpha;
        // beta = -beta;
        // gamma = -gamma;

        return new double[] { result.get(0, 0), result.get(1, 0), result.get(2, 0) };

    }
}
