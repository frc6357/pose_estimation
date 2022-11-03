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

    /** Creates a new ExampleSubsystem. */
    public IMUSubsystem() {
    }

    @Override
    public void periodic() {
        imu.addDataPoint();

        alpha = imu.getYaw();
        beta = imu.getRoll();
        gamma = imu.getPitch();

        double[] globalAccel = getGlobalAccel();

        kalmanX.predict(globalAccel[0]);
        kalmanY.predict(globalAccel[1]);

        SmartDashboard.putNumber("Yaw", imu.getAngle());

        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Complementary X", imu.getXComplementaryAngle());
        SmartDashboard.putNumber("Complementary Y", imu.getYComplementaryAngle());

        SmartDashboard.putNumber("Filtered X Accel Angle", imu.getXFilteredAccelAngle());
        SmartDashboard.putNumber("Filtered Y Accel Angle", imu.getYFilteredAccelAngle());

        SmartDashboard.putNumber("X Accel", imu.getAccelX());
        SmartDashboard.putNumber("Y Accel", imu.getAccelY());

        SmartDashboard.putNumber("X Position", kalmanX.getState());
        SmartDashboard.putNumber("Y Position", kalmanY.getState());

        SmartDashboard.putNumber("Pitch", beta);
        SmartDashboard.putNumber("Roll", gamma);

        SmartDashboard.putNumber("Global X", globalAccel[0]);
        SmartDashboard.putNumber("Global Y", globalAccel[1]);
        SmartDashboard.putNumber("Global Z", globalAccel[2]);
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
                { { imu.getAccelX() },
                { imu.getAccelY() },
                { imu.getAccelZ() } }));

        Matrix<N3, N1> result = yawMatrix.times(pitchMatrix).times(rollMatrix).times(accelMatrix);

        return new double[] { result.get(0, 0), result.get(0, 1), result.get(0, 2) };

    }
}
