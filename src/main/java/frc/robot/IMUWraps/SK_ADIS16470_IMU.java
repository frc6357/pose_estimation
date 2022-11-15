// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.IMUWraps;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;

/**
 * An IMU class that extends the ADIS16470_IMU and determines the acceleration
 * in each axis and filters out the acceleration of gravity based on the
 * calibration data
 */
public class SK_ADIS16470_IMU extends ADIS16470_IMU {

    /** The value of gravity */
    private double gravity = 9.81;
    /** The initial yaw of the robot set by the calibration method */
    private double initYaw = 0;
    /** The yaw that the imu is offset compared the robot front */
    private double offsetYaw = 0;

    /** Average acceleration in each axis {x, y, z} at startup */
    private double[] calib = new double[3];
    /** Number of data points needed for calibration */
    private int calibPoints = 250;

    /** Moving average data set */
    private double[][] mvgAvg = new double[calibPoints][3];
    /** The current index that will be updated for the mvgAvg array */
    private int currentIndex = 0;

    /**
     * Creates a new IMU using the ADIS16470_IMU class.
     * 
     * @param yaw_axis The axis that measures the yaw
     * @param port     The SPI Port the gyro is plugged into
     * @param cal_time Calibration time
     */
    public SK_ADIS16470_IMU(IMUAxis yaw_axis, SPI.Port port, CalibrationTime cal_time) {
        super(yaw_axis, port, cal_time);
        axisCalibrate();
    }

    /** Creates a new IMU using the ADIS16470_IMU class. */
    public SK_ADIS16470_IMU() {
        super();
        axisCalibrate();
    }

    /**
     * Calibrate the gyro. It's important to make sure that the robot is not moving
     * while the calibration is in progress, this is typically done when the robot
     * is first turned on while it's sitting at rest before the match starts. This
     * will find the average acceleration in each axis
     * 
     * @return An array containing the average acceleration in each axis {x, y, z}
     *         at calibration time
     */
    private void axisCalibrate() {

        // Resetting the calibration values
        calib[0] = 0;
        calib[1] = 0;
        calib[2] = 0;

        // Start collecting data to find average acceleration while standing still
        for (int i = 0; i < calibPoints; i++) {

            // Adding all the acceleration values
            // on an axis by axis basis
            mvgAvg[i][0] = getAccelX();
            mvgAvg[i][1] = getAccelY();
            mvgAvg[i][2] = getAccelZ();

            // Waiting 20 ms before collecting next set of data
            Timer.delay(0.02);
        }

        averageData();
        calculateGravity();
    }

    /**
     * Reads the acceleration data from the IMU when called and adds it to the
     * moving average data set. Only call once per schedule run.
     */
    public void addDataPoint() {
        mvgAvg[currentIndex][0] = getAccelX();
        mvgAvg[currentIndex][1] = getAccelY();
        mvgAvg[currentIndex][2] = getAccelZ();

        currentIndex++;
        currentIndex = (currentIndex >= calibPoints) ? 0 : currentIndex;
    }

    public void averageData() {
        double sumX = 0;
        double sumY = 0;
        double sumZ = 0;

        // Dividing the sums of the acceleration to find the average acceleration
        // in each axis.
        for (int i = 0; i < calibPoints; i++) {
            sumX += mvgAvg[i][0];
            sumY += mvgAvg[i][1];
            sumZ += mvgAvg[i][2];
        }

        calib[0] = sumX / calibPoints;
        calib[1] = sumY / calibPoints;
        calib[2] = sumZ / calibPoints;
    }

    /**
     * Determines the measurable value of gravity. This should be done after the IMU
     * has been calibrated while standing still
     */
    private void calculateGravity() {
        // Calculating the magnitude of gravity
        gravity = Math.sqrt(
                Math.pow(calib[0], 2) +
                        Math.pow(calib[1], 2) +
                        Math.pow(calib[2], 2));
    }

    public double getRateX() {
        return m_gyro_rate_x;
    }

    public double getRateY() {
        return m_gyro_rate_y;
    }

    public double getRateZ() {
        return m_gyro_rate_z;
    }

    /**
     * Finds the absolute pitch of the imu after calibration
     * @return The pitch angle in degrees
     */
    public double getPitch()
    {
        return getYComplementaryAngle();
    }

    /**
     * Finds the absolute roll of the imu after calibration 
     * @return The roll angle in degrees
     */
    public double getRoll()
    {
        return -getXComplementaryAngle();
    }

    /**
     * Find the yaw of the imu after the offset has been applied
     * @return The yaw angle in degrees
     */
    public double getYaw()
    {
        return getAngle() + offsetYaw;
    }

    /**
     * Zeroes the yaw angle by measuring the current read angle
     */
    public void calibrateYaw()
    {
        offsetYaw = -getAngle();
    }

}
