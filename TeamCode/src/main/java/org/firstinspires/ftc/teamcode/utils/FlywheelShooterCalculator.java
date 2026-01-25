package org.firstinspires.ftc.teamcode.utils;

import android.annotation.SuppressLint;

public class FlywheelShooterCalculator {

    // --- Physical Constants ---
    private static final double GRAVITY = 9.81;
    private static final double AIR_DENSITY = 1.225; // kg/m^3 (Standard sea level)

    // --- Flywheel & Projectile Properties ---
    private static final double FLYWHEEL_DIAMETER_M = 0.09;
    private static final double FLYWHEEL_RADIUS_M = FLYWHEEL_DIAMETER_M / 2.0;
    private static final double FLYWHEEL_MOI = 0.0004682234;

    private static final double PROJECTILE_WEIGHT_KG = 0.075;
    private static final double PROJECTILE_RADIUS_M = 0.0635;

    // NEW: Drag Properties
    private static final double DRAG_COEFFICIENT = 0.6;
    // Area = pi * r^2
    private static final double PROJECTILE_AREA = Math.PI * Math.pow(PROJECTILE_RADIUS_M, 2);

    // --- Optimization Constraints ---
    private static final double MIN_HOOD_ANGLE_DEG = 45.0;
    private static final double MAX_HOOD_ANGLE_DEG = 80.0;

    // --- Simulation Configuration ---
    private static final double TIME_STEP = 0.001; // 1ms accuracy
    private static final double MAX_SEARCH_VELOCITY = 60.0; // m/s limit for safety

    /**
     * Loops through angles and uses Binary Search to find the velocity required for each.
     */
    public static ShootingSolution findOptimalShootingSolution(double targetX, double targetY) {
        ShootingSolution bestSolution = null;
        double minExitVelocity = Double.MAX_VALUE;

        // Step through angles
        for (double angleDeg = MIN_HOOD_ANGLE_DEG; angleDeg <= MAX_HOOD_ANGLE_DEG; angleDeg += 0.5) {

            // Find velocity needed to hit target at this angle using Binary Search
            double requiredVel = findVelocityForAngle(angleDeg, targetX, targetY);

            // Optimization: Keep the solution with the lowest required velocity (energy)
            if (requiredVel > 0 && requiredVel < minExitVelocity) {
                minExitVelocity = requiredVel;
                double rpm = calculateFlywheelRPM(requiredVel);
                bestSolution = new ShootingSolution(angleDeg, requiredVel, rpm);
            }
        }
        return bestSolution;
    }

    /**
     * Binary Search: Guesses a velocity, simulates the shot, corrects the guess.
     */
    private static double findVelocityForAngle(double angleDeg, double targetX, double targetY) {
        double low = 0.0;
        double high = MAX_SEARCH_VELOCITY;
        double angleRad = Math.toRadians(angleDeg);

        // 25 iterations gives extreme precision
        for (int i = 0; i < 25; i++) {
            double midVel = low + (high - low) / 2;

            // Run the physics engine
            double heightAtTarget = simulateShot(midVel, angleRad, targetX);

            if (heightAtTarget < targetY) {
                low = midVel; // Shot too low, speed up
            } else {
                high = midVel; // Shot too high, slow down
            }
        }

        // Verify the result is actually close (within 5cm)
        if (Math.abs(simulateShot(high, angleRad, targetX) - targetY) > 0.05) {
            return -1.0; // Invalid
        }

        return high;
    }

    /**
     * The Physics Engine: Simulates ball flight step-by-step with Drag.
     * Returns the height (Y) of the ball when it reaches the target distance (X).
     */
    private static double simulateShot(double vStart, double angleRad, double targetX) {
        double x = 0;
        double y = 0;
        double vx = vStart * Math.cos(angleRad);
        double vy = vStart * Math.sin(angleRad);

        // Simulation Loop
        while (x < targetX && y > -5.0) { // Stop if we pass target or hit ground
            double vTotal = Math.sqrt(vx*vx + vy*vy);

            // --- DRAG CALCULATION ---
            // Fd = 0.5 * rho * v^2 * Cd * A
            double dragForce = 0.5 * AIR_DENSITY * (vTotal * vTotal) * DRAG_COEFFICIENT * PROJECTILE_AREA;

            // Break Drag into components (opposes motion)
            // theta = atan2(vy, vx) -> cos = vx/v, sin = vy/v
            double dragForceX = dragForce * (vx / vTotal);
            double dragForceY = dragForce * (vy / vTotal);

            // F = ma -> a = F/m
            double ax = -(dragForceX / PROJECTILE_WEIGHT_KG);
            double ay = -GRAVITY - (dragForceY / PROJECTILE_WEIGHT_KG);

            // Update Velocity & Position (Euler Integration)
            vx += ax * TIME_STEP;
            vy += ay * TIME_STEP;

            x += vx * TIME_STEP;
            y += vy * TIME_STEP;
        }

        return y;
    }

    /**
     * Calculates RPM using your MOI efficiency logic.
     */
    private static double calculateFlywheelRPM(double desiredExitVelocity) {
        double surfaceSpeedMPS = 2.0 * desiredExitVelocity;

        // Efficiency Factor using Conservation of Momentum approximation
        double efficiencyFactor = FLYWHEEL_MOI / (FLYWHEEL_MOI + PROJECTILE_WEIGHT_KG * Math.pow(FLYWHEEL_RADIUS_M, 2));

        double compensatedSpeed = surfaceSpeedMPS / efficiencyFactor;

        // Convert to RPM
        return (compensatedSpeed / (2 * Math.PI * FLYWHEEL_RADIUS_M)) * 60;
    }

    public static class ShootingSolution {
        double hoodAngleDeg;
        double exitVelocity;
        double flywheelRPM;

        public ShootingSolution(double angle, double vel, double rpm) {
            this.hoodAngleDeg = angle;
            this.exitVelocity = vel;
            this.flywheelRPM = rpm;
        }
    }
}