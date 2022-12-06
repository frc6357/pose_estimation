// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.IMUWraps.SK_ADIS16470_IMU;

public class IMUSubsystem extends SubsystemBase {
    private SK_ADIS16470_IMU imu = new SK_ADIS16470_IMU();

    private long counter = 0;

    private double[] globalAccel;

    private boolean isEnabled = false;

    @Override
    public void periodic() {
        imu.addDataPoint();

        // This is for quick resets and is to be used for testing only
        // TODO: Remove this once tested
        // Reset the Kalman Filter Position and Velocity if the robot becomes enabled
        boolean stateChanged = isEnabled != DriverStation.isEnabled();
        isEnabled = stateChanged ? DriverStation.isEnabled() : isEnabled;
        if (stateChanged && isEnabled == true)
        {
            imu.resetKalmanFilter();
        }



        // This method will be called once per scheduler run
        
        SmartDashboard.putNumber("X Position", imu.getPositionX());
        SmartDashboard.putNumber("X Velocity", imu.getVelocityX());
        SmartDashboard.putNumber("Y Position", imu.getPositionY());
        SmartDashboard.putNumber("Y Velocity", imu.getVelocityY());

        globalAccel = imu.getGlobalAccel();

        SmartDashboard.putNumber("Global X", globalAccel[0]);
        SmartDashboard.putNumber("Global Y", globalAccel[1]);
        SmartDashboard.putNumber("Global Z", globalAccel[2]);

        // SmartDashboard.putNumber("Delta Z Accel", globalAccel[2] - zAccel);
        // SmartDashboard.putNumber("Global Accel Residual", 9.81-(Math.sqrt((globalAccel[0]*globalAccel[0])+(globalAccel[1]*globalAccel[1])+(globalAccel[2]*globalAccel[2]))));

        // double preRotated = Math.sqrt(Math.pow(xAccel, 2) + Math.pow(yAccel, 2) + Math.pow(zAccel, 2));
        // double postRotated = Math.sqrt(Math.pow(globalAccel[0], 2) + Math.pow(globalAccel[1], 2) + Math.pow(globalAccel[2], 2));

        // SmartDashboard.putNumber("Pre-Rotated Magnitude", preRotated);
        // SmartDashboard.putNumber("Rotated Magnitude", postRotated);
        // SmartDashboard.putNumber("Delta Magnitude", postRotated - preRotated);

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
}
