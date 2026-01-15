package org.firstinspires.ftc.teamcode.utils;

public class ShooterCalculator {

    // Calibration range
    private static final double MIN_DIST = 60.0;
    private static final double MAX_DIST = 310.0;

    /* -------------------------
       Hood (quadratic) coefficients
       hood(x) = a*x^2 + b*x + c
       ------------------------- */
    private static final double HOOD_A = -2.53673938e-06;
    private static final double HOOD_B = 0.00151446382;
    private static final double HOOD_C = 0.124335198;

    /* -------------------------
       Velocity (cubic) coefficients
       vel(x) = p*x^3 + q*x^2 + r*x + s
       ------------------------- */
    private static final double VEL_P = 2.61759983e-05;
    private static final double VEL_Q = -0.00657247599;
    private static final double VEL_R = 0.535241586;
    private static final double VEL_S = 1285.88083;

    // Utility: clamp
    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    // Evaluate hood polynomial (input in cm)
    public static double evalHoodPoly(double distanceCm) {
        double x = clamp(distanceCm, MIN_DIST, MAX_DIST);
        return HOOD_A*x*x + HOOD_B*x + HOOD_C;
    }

    // Evaluate velocity polynomial (input in cm)
    public static double evalVelPoly(double distanceCm) {
        double x = clamp(distanceCm, MIN_DIST, MAX_DIST);
        return VEL_P*x*x*x + VEL_Q*x*x + VEL_R*x + VEL_S;
    }

    // Main API: compute ShotPoint for any distance
    public static ShotPoint calculateShot(double distanceCm) {
        // Option: clamp input before computing to avoid extrapolation
        double d = clamp(distanceCm, MIN_DIST, MAX_DIST);

        double hood = evalHoodPoly(d);
        double vel  = evalVelPoly(d);

        // Optional: enforce hood range (0..1) and sensible velocity bounds
        if (hood < 0.0) hood = 0.0;
        if (hood > 1.0) hood = 1.0;
        if (vel < 0) vel = 0;

        return new ShotPoint(distanceCm, hood, vel);
    }

    /* ============================
       Optional: piecewise linear
       If you prefer exact matches at calibration points,
       implement interpolation using your calibration list.
       ============================ */

}

