package org.firstinspires.ftc.teamcode.utils;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class ShooterRegression {

    // --- Base Distance Checkpoints (139" completely replaced with 130") ---
    private static final double DIST_ULTRA_CLOSE = 48.6;
    private static final double DIST_52_INCH = 52.0;
    private static final double DIST_75_INCH = 75.0;
    private static final double DIST_98_INCH = 98.0;
    private static final double DIST_114_INCH = 114.0;
    private static final double DIST_130_INCH = 130.0;     // <-- New node replacing 139"
    private static final double DIST_DEEP_FAR = 155.0;

    // --- Global Offsets ---
    public static double GLOBAL_VELOCITY_OFFSET = 0.0;

    // --- Node-Specific Velocity Offsets (Configurable via Dashboard) ---
    public static double OFFSET_ULTRA_CLOSE = 0.0;
    public static double OFFSET_52_INCH = 0.0;
    public static double OFFSET_75_INCH = 0.0;
    public static double OFFSET_98_INCH = 0.0;
    public static double OFFSET_114_INCH = 0.0;
    public static double OFFSET_130_INCH = 0.0;           // <-- New offset tracking hook
    public static double OFFSET_DEEP_FAR = 0.0;

    /**
     * Helper class to manage data nodes and dynamically calculate linear equations
     */
    private static class ZoneData {
        final double dist, y1, x1, y2, x2, minClamp;
        ZoneData(double d, double y1, double x1, double y2, double x2, double minClamp) {
            this.dist = d;
            this.y1 = y1; // lowHood
            this.x1 = x1; // lowVel
            this.y2 = y2; // highHood
            this.x2 = x2; // highVel
            this.minClamp = minClamp;
        }
    }

    /**
     * Calculates the dynamic hood angle percentage based on distance and CURRENT actual Ticks/s.
     * Uses velocity as the X-axis and Hood Pct (0-100) as the Y-axis.
     */
    public double calculateDynamicHood(double distance, double currentTicksPerSec) {
        // Explicitly mapping dashboard calibration parameters
        ZoneData zUltraClose = new ZoneData(DIST_ULTRA_CLOSE,  0.0, 1100.0,  70.0, 1220.0,  0.0);
        ZoneData z52Inch    = new ZoneData(DIST_52_INCH,      0.0, 1000.0,  60.0, 1140.0,  0.0);
        ZoneData z75Inch     = new ZoneData(DIST_75_INCH,     20.0, 1120.0, 100.0, 1260.0, 20.0);
        ZoneData z98Inch     = new ZoneData(DIST_98_INCH,     20.0, 1160.0, 100.0, 1250.0, 20.0);
        ZoneData z114Inch    = new ZoneData(DIST_114_INCH,    20.0, 1240.0, 100.0, 1300.0, 20.0);

        // 130-INCH DATA: 40% hood @ 1420 ticks/s -> 100% hood @ 1500 ticks/s
        ZoneData z130Inch    = new ZoneData(DIST_130_INCH,    40.0, 1420.0, 100.0, 1500.0, 40.0);

        ZoneData zDeepFar    = new ZoneData(DIST_DEEP_FAR,    50.0, 1440.0, 100.0, 1525.0, 50.0);

        ZoneData zStart, zEnd;
        double offsetStart, offsetEnd;

        // 1. Identify active distance bracket
        if (distance <= DIST_52_INCH) {
            zStart = zUltraClose; offsetStart = OFFSET_ULTRA_CLOSE;
            zEnd = z52Inch;       offsetEnd = OFFSET_52_INCH;
        } else if (distance <= DIST_75_INCH) {
            zStart = z52Inch;     offsetStart = OFFSET_52_INCH;
            zEnd = z75Inch;       offsetEnd = OFFSET_75_INCH;
        } else if (distance <= DIST_98_INCH) {
            zStart = z75Inch;     offsetStart = OFFSET_75_INCH;
            zEnd = z98Inch;       offsetEnd = OFFSET_98_INCH;
        } else if (distance <= DIST_114_INCH) {
            zStart = z98Inch;     offsetStart = OFFSET_98_INCH;
            zEnd = z114Inch;      offsetEnd = OFFSET_114_INCH;
        } else if (distance <= DIST_130_INCH) {
            zStart = z114Inch;    offsetStart = OFFSET_114_INCH;
            zEnd = z130Inch;      offsetEnd = OFFSET_130_INCH;
        } else {
            zStart = z130Inch;    offsetStart = OFFSET_130_INCH;
            zEnd = zDeepFar;      offsetEnd = OFFSET_DEEP_FAR;
        }

        // 2. Interpolate the target lines boundaries across distances
        double x1 = interpolate(distance, zStart.dist, zEnd.dist, zStart.x1, zEnd.x1) + GLOBAL_VELOCITY_OFFSET + offsetStart;
        double x2 = interpolate(distance, zStart.dist, zEnd.dist, zStart.x2, zEnd.x2) + GLOBAL_VELOCITY_OFFSET + offsetEnd;

        double y1 = interpolate(distance, zStart.dist, zEnd.dist, zStart.y1, zEnd.y1);
        double y2 = interpolate(distance, zStart.dist, zEnd.dist, zStart.y2, zEnd.y2);

        // 3. Slope (m) and Intercept (b) calculation where Hood = m * Velocity + b
        double m = (y2 - y1) / (x2 - x1);
        double b = y1 - (m * x1);

        // 4. Determine safe bounds constraints
        double minHoodClamp = interpolate(distance, zStart.dist, zEnd.dist, zStart.minClamp, zEnd.minClamp);
        double maxHoodClamp = 100.0;

        // 5. Compute output percentage
        double calculatedAngle = (m * currentTicksPerSec) + b;

        return Math.max(minHoodClamp, Math.min(maxHoodClamp, calculatedAngle));
    }

    /**
     * Pure Linear Interpolation target velocity tracking high-hood endpoints.
     */
    public double getTargetRpm(double distance) {
        double targetSpeed;

        if (distance <= DIST_52_INCH) {
            targetSpeed = interpolate(distance, DIST_ULTRA_CLOSE, DIST_52_INCH, 1220.0, 1140.0);
        } else if (distance <= DIST_75_INCH) {
            targetSpeed = interpolate(distance, DIST_52_INCH, DIST_75_INCH, 1140.0, 1260.0);
        } else if (distance <= DIST_98_INCH) {
            targetSpeed = interpolate(distance, DIST_75_INCH, DIST_98_INCH, 1260.0, 1250.0);
        } else if (distance <= DIST_114_INCH) {
            targetSpeed = interpolate(distance, DIST_98_INCH, DIST_114_INCH, 1250.0, 1300.0);
        } else if (distance <= DIST_130_INCH) {
            targetSpeed = interpolate(distance, DIST_114_INCH, DIST_130_INCH, 1300.0, 1500.0);
        } else {
            targetSpeed = interpolate(distance, DIST_130_INCH, DIST_DEEP_FAR, 1500.0, 1525.0);
        }

        double currentSpecificOffset = getInterpolatedOffset(distance);
        return targetSpeed + GLOBAL_VELOCITY_OFFSET + currentSpecificOffset;
    }

    /**
     * Pure Linear Interpolation minimum safety velocity tracking low-hood floor.
     */
    public double getMinRpmForDistance(double distance) {
        double minSpeed;

        if (distance <= DIST_52_INCH) {
            minSpeed = interpolate(distance, DIST_ULTRA_CLOSE, DIST_52_INCH, 1100.0, 1000.0);
        } else if (distance <= DIST_75_INCH) {
            minSpeed = interpolate(distance, DIST_52_INCH, DIST_75_INCH, 1000.0, 1120.0);
        } else if (distance <= DIST_98_INCH) {
            minSpeed = interpolate(distance, DIST_75_INCH, DIST_98_INCH, 1120.0, 1160.0);
        } else if (distance <= DIST_114_INCH) {
            minSpeed = interpolate(distance, DIST_98_INCH, DIST_114_INCH, 1160.0, 1240.0);
        } else if (distance <= DIST_130_INCH) {
            minSpeed = interpolate(distance, DIST_114_INCH, DIST_130_INCH, 1240.0, 1420.0);
        } else {
            minSpeed = interpolate(distance, DIST_130_INCH, DIST_DEEP_FAR, 1420.0, 1440.0);
        }

        double currentSpecificOffset = getInterpolatedOffset(distance);
        return minSpeed + GLOBAL_VELOCITY_OFFSET + currentSpecificOffset;
    }

    /**
     * Helper to smoothly transition between node-specific offsets depending on where the robot is.
     */
    private double getInterpolatedOffset(double distance) {
        if (distance <= DIST_52_INCH) {
            return interpolate(distance, DIST_ULTRA_CLOSE, DIST_52_INCH, OFFSET_ULTRA_CLOSE, OFFSET_52_INCH);
        } else if (distance <= DIST_75_INCH) {
            return interpolate(distance, DIST_52_INCH, DIST_75_INCH, OFFSET_52_INCH, OFFSET_75_INCH);
        } else if (distance <= DIST_98_INCH) {
            return interpolate(distance, DIST_75_INCH, DIST_98_INCH, OFFSET_75_INCH, OFFSET_98_INCH);
        } else if (distance <= DIST_114_INCH) {
            return interpolate(distance, DIST_98_INCH, DIST_114_INCH, OFFSET_98_INCH, OFFSET_114_INCH);
        } else if (distance <= DIST_130_INCH) {
            return interpolate(distance, DIST_114_INCH, DIST_130_INCH, OFFSET_114_INCH, OFFSET_130_INCH);
        } else {
            return interpolate(distance, DIST_130_INCH, DIST_DEEP_FAR, OFFSET_130_INCH, OFFSET_DEEP_FAR);
        }
    }

    // Standard linear interpolation helper
    private double interpolate(double input, double inputMin, double inputMax, double outputMin, double outputMax) {
        if (input <= inputMin) return outputMin;
        if (input >= inputMax) return outputMax;
        return outputMin + ((input - inputMin) / (inputMax - inputMin)) * (outputMax - outputMin);
    }
}