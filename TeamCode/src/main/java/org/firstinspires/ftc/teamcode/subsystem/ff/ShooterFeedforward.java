package org.firstinspires.ftc.teamcode.subsystem.ff;

import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedforward.FeedforwardElement;

public class ShooterFeedforward implements FeedforwardElement {
    // https://curve.fit/KjezG82L/single/20260130064210
    private static final double a = 2.824e-10;
    private static final double b = -7.497e-07;
    private static final double c = 1.013e-03;
    private static final double d = -7.218e-02;

    @Override
    public double calculate(KineticState reference) {
        double speed = reference.getVelocity() + 80; // Your +80 offset
        return ((a * speed + b) * speed + c) * speed + d;
    }

    @Override
    public void reset() {
        // Nothing to reset for this static feedforward
    }
}