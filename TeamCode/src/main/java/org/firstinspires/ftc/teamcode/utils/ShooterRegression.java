package org.firstinspires.ftc.teamcode.utils;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class ShooterRegression {

    // --- Base Distance Checkpoints ---
    private static final double DIST_ULTRA_CLOSE = 48.6;
    private static final double DIST_CLOSE = 56.3;
    private static final double DIST_MID = 82.15;
    private static final double DIST_FAR = 128.07;
    private static final double DIST_DEEP_FAR = 155.0;

    // --- Global Offsets ---
    public static double GLOBAL_VELOCITY_OFFSET = -20;

    // --- Node-Specific Velocity Offsets (Configurable via Dashboard) ---
    public static double OFFSET_ULTRA_CLOSE = 0.0;
    public static double OFFSET_CLOSE = 0.0;
    public static double OFFSET_MID = 0.0;
    public static double OFFSET_FAR = 0.0;
    public static double OFFSET_DEEP_FAR = 0.0;

    /**
     * Helper class to manage data nodes and dynamically calculate linear equations
     */
    private static class DataPoint {
        double distance;
        double baseTargetVelocity;
        double hoodPercentage;
        double minHoodClamp;

        public DataPoint(double distance, double baseTargetVelocity, double hoodPercentage, double minHoodClamp) {
            this.distance = distance;
            this.baseTargetVelocity = baseTargetVelocity;
            this.hoodPercentage = hoodPercentage;
            this.minHoodClamp = minHoodClamp;
        }

        // Returns the target velocity adjusted by both the global and its node-specific offset
        public double getAdjustedVelocity(double nodeSpecificOffset) {
            return this.baseTargetVelocity + GLOBAL_VELOCITY_OFFSET + nodeSpecificOffset;
        }
    }

    /**
     * Calculates the dynamic hood angle percentage based on distance and CURRENT actual Ticks/s.
     * Recalculates slopes and intercepts dynamically to respect independent node velocity offsets.
     */
    public double calculateDynamicHood(double distance, double currentTicksPerSec) {
        // Reverse-engineered base target velocities from your original m and b constants
        // (Derived via: Target Velocity = (Hood% - b) / m)
        DataPoint pUltraClose = new DataPoint(DIST_ULTRA_CLOSE, 1100.0, 0.5833, 0.0);
        DataPoint pClose      = new DataPoint(DIST_CLOSE,      1000.0, 0.2000, 20.0);
        DataPoint pMid        = new DataPoint(DIST_MID,        2133.3, 0.5000, 0.0);
        DataPoint pFar        = new DataPoint(DIST_FAR,        2260.0, 0.4615, 40.0);
        DataPoint pDeepFar    = new DataPoint(DIST_DEEP_FAR,   2290.0, 0.5882, 50.0);

        DataPoint pStart, pEnd;
        double offsetStart, offsetEnd;

        // 1. Determine which distance zone we are currently operating in
        if (distance <= DIST_CLOSE) {
            pStart = pUltraClose; offsetStart = OFFSET_ULTRA_CLOSE;
            pEnd = pClose;        offsetEnd = OFFSET_CLOSE;
        } else if (distance <= DIST_MID) {
            pStart = pClose;      offsetStart = OFFSET_CLOSE;
            pEnd = pMid;          offsetEnd = OFFSET_MID;
        } else if (distance <= DIST_FAR) {
            pStart = pMid;        offsetStart = OFFSET_MID;
            pEnd = pFar;          offsetEnd = OFFSET_FAR;
        } else {
            pStart = pFar;        offsetStart = OFFSET_FAR;
            pEnd = pDeepFar;      offsetEnd = OFFSET_DEEP_FAR;
        }

        // 2. Get the actual velocity X-coordinates for the line formula, including offsets
        double x1 = pStart.getAdjustedVelocity(offsetStart);
        double x2 = pEnd.getAdjustedVelocity(offsetEnd);

        // Y-coordinates remain your fixed structural physical hood constraints
        double y1 = pStart.hoodPercentage;
        double y2 = pEnd.hoodPercentage;

        // 3. Dynamically calculate slope (m) and intercept (b) based on the offset nodes
        double m = (y2 - y1) / (x2 - x1);
        double b = y1 - (m * x1);

        // 4. Interpolate the minimum hood clamp based on distance
        double minHoodClamp = interpolate(distance, pStart.distance, pEnd.distance, pStart.minHoodClamp, pEnd.minHoodClamp);
        double maxHoodClamp = 100.0;

        // 5. Calculate the final hood angle
        double calculatedAngle = (m * currentTicksPerSec) + b;

        return Math.max(minHoodClamp, Math.min(maxHoodClamp, calculatedAngle));
    }

    /**
     * Calculates ideal Target Ticks/s using a smooth Quadratic Polynomial regression curve (R^2 = 0.9722).
     */
    public double getTargetRpm(double distance) {
        double boundedDistance = Math.max(DIST_ULTRA_CLOSE, Math.min(DIST_DEEP_FAR, distance));

        double targetSpeed = (0.001449 * Math.pow(boundedDistance, 2))
                + (2.9401 * boundedDistance)
                + 1049.3;

        // Node-specific offset interpolation for the target RPM output
        double currentSpecificOffset = getInterpolatedOffset(distance);

        return Math.max(1200.0, Math.min(1525.0, targetSpeed)) + GLOBAL_VELOCITY_OFFSET + currentSpecificOffset;
    }

    /**
     * Calculates the minimum Ticks/s required to clear the shot safely using a Quadratic Fit (R^2 = 0.9304).
     */
    public double getMinRpmForDistance(double distance) {
        double boundedDistance = Math.max(DIST_ULTRA_CLOSE, Math.min(DIST_DEEP_FAR, distance));

        double minSpeed = (0.02810 * Math.pow(boundedDistance, 2))
                - (1.7613 * boundedDistance)
                + 1063.4;

        double currentSpecificOffset = getInterpolatedOffset(distance);

        return Math.max(1000.0, Math.min(1440.0, minSpeed)) + GLOBAL_VELOCITY_OFFSET + currentSpecificOffset;
    }

    /**
     * Helper to smoothly transition between node-specific offsets depending on where the robot is.
     */
    private double getInterpolatedOffset(double distance) {
        if (distance <= DIST_CLOSE) {
            return interpolate(distance, DIST_ULTRA_CLOSE, DIST_CLOSE, OFFSET_ULTRA_CLOSE, OFFSET_CLOSE);
        } else if (distance <= DIST_MID) {
            return interpolate(distance, DIST_CLOSE, DIST_MID, OFFSET_CLOSE, OFFSET_MID);
        } else if (distance <= DIST_FAR) {
            return interpolate(distance, DIST_MID, DIST_FAR, OFFSET_MID, OFFSET_FAR);
        } else {
            return interpolate(distance, DIST_FAR, DIST_DEEP_FAR, OFFSET_FAR, OFFSET_DEEP_FAR);
        }
    }

    // Standard linear interpolation helper
    private double interpolate(double input, double inputMin, double inputMax, double outputMin, double outputMax) {
        if (input <= inputMin) return outputMin;
        if (input >= inputMax) return outputMax;
        return outputMin + ((input - inputMin) / (inputMax - inputMin)) * (outputMax - outputMin);
    }
}