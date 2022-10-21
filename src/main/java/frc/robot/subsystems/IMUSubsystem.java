// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.IMUWraps.SK_ADIS16470_IMU;
import frc.robot.tools.KalmanPose;

public class IMUSubsystem extends SubsystemBase {
    private SK_ADIS16470_IMU imu = new SK_ADIS16470_IMU();

    private KalmanPose kalmanX = new KalmanPose();
    private KalmanPose kalmanY = new KalmanPose();

    private double alpha = 0.0;
    private double beta = 0.0;
    private double gamma = 0.0;

    /** Creates a new ExampleSubsystem. */
    public IMUSubsystem() {
    }

    @Override
    public void periodic() {

        alpha = imu.getYaw();
        beta = imu.getPitch();
        gamma = imu.getRoll();

        kalmanX.predict(getGlobalXAccel());
        kalmanY.predict(getGlobalYAccel());

        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Complementary X", imu.getXComplementaryAngle());
        SmartDashboard.putNumber("Complementary Y", imu.getYComplementaryAngle());

        SmartDashboard.putNumber("Filtered X Accel Angle", imu.getXFilteredAccelAngle());
        SmartDashboard.putNumber("Filtered Y Accel Angle", imu.getYFilteredAccelAngle());

        SmartDashboard.putNumber("X Position", kalmanX.getState());
        SmartDashboard.putNumber("Y Position", kalmanY.getState());
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    public double getGlobalXAccel()
    {
        return  imu.getAccelX() *
                    (Math.cos(alpha) * Math.cos(beta)) +
                imu.getAccelY() *
                    (Math.sin(alpha) * Math.sin(beta) * Math.cos(gamma)
                    - Math.cos(alpha) * Math.sin(gamma)) +
                imu.getAccelZ() *
                    (Math.cos(alpha) * Math.sin(beta) + Math.cos(gamma)
                    + Math.sin(alpha) * Math.sin(gamma));
    }

    public double getGlobalYAccel()
    {
        return  imu.getAccelX() *
                    (Math.cos(alpha) * Math.sin(beta)) +
                imu.getAccelY() *
                    (Math.sin(alpha) * Math.sin(beta) * Math.sin(gamma)
                    + Math.cos(alpha) * Math.cos(gamma)) +
                imu.getAccelZ() *
                    (Math.cos(alpha) * Math.sin(beta) + Math.sin(gamma)
                    - Math.sin(alpha) * Math.cos(gamma));
    }
}
