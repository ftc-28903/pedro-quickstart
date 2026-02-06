package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.subsystem.Transfer;
import org.firstinspires.ftc.teamcode.subsystem.Webcam;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.control.feedforward.BasicFeedforwardParameters;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.groups.CommandGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.IMUEx;
import dev.nextftc.hardware.impl.MotorEx;

@Configurable
public class TestAutoFactory {
    private final MotorEx frontLeftMotor;
    private final MotorEx frontRightMotor;
    private final MotorEx backLeftMotor;
    private final MotorEx backRightMotor;
    private final IMUEx imu;

    private double continuousHeading = 0;
    private double lastImuAngle = 0;
    private boolean firstHeadingUpdate = true;

    // For strafing, we'll track the "average" position of relevant motors
    private double lastStrafePosition = 0;
    private boolean firstStrafeUpdate = true;

    public int dirMultiplier = 1;

    public static PIDCoefficients drivePIDCoefficients = new PIDCoefficients(0.03, 0, 0.0);
    public static BasicFeedforwardParameters driveFeedforwardParameters = new BasicFeedforwardParameters(0,0,0);
    private ControlSystem drivePID = ControlSystem.builder()
            .posPid(drivePIDCoefficients)
            .basicFF(driveFeedforwardParameters)
            .build();

    // Separate PID for strafing (can use same coefficients or different ones)
    public static PIDCoefficients strafePIDCoefficients = new PIDCoefficients(0.03, 0, 0.0);
    public static BasicFeedforwardParameters strafeFeedforwardParameters = new BasicFeedforwardParameters(0,0,0);
    private ControlSystem strafePID = ControlSystem.builder()
            .posPid(strafePIDCoefficients)
            .basicFF(strafeFeedforwardParameters)
            .build();

    public static PIDCoefficients headingPIDCoefficients = new PIDCoefficients(0.033, 0, 0.0);
    public static BasicFeedforwardParameters headingFeedforwardParameters = new BasicFeedforwardParameters(0,0,0);
    private ControlSystem headingPID = ControlSystem.builder()
            .posPid(headingPIDCoefficients)
            .basicFF(headingFeedforwardParameters)
            .build();

    public TestAutoFactory(MotorEx frontLeftMotor, MotorEx frontRightMotor, MotorEx backLeftMotor, MotorEx backRightMotor, IMUEx imu) {
        this.frontLeftMotor = frontLeftMotor;
        this.frontRightMotor = frontRightMotor;
        this.backLeftMotor = backLeftMotor;
        this.backRightMotor = backRightMotor;
        this.imu = imu;
    }

    public void runAllMotors(double fl, double fr, double bl, double br) {
        frontLeftMotor.setPower(fl);
        frontRightMotor.setPower(fr);
        backLeftMotor.setPower(bl);
        backRightMotor.setPower(br);
    }

    public double getForwardPosition() {
        return (
                backLeftMotor.getCurrentPosition() +
                        backRightMotor.getCurrentPosition()
        ) / 2.0;
    }

    public double getStrafePosition() {
        // For strafing with mecanum wheels, we can use the average of front right and back left motors
        // or front left and back right, depending on your strafe direction convention
        // This assumes:
        // - Strafing right: frontLeft and backRight go forward, frontRight and backLeft go backward
        // - Strafing left: frontLeft and backRight go backward, frontRight and backLeft go forward

        // Using average of two opposing wheels that move together during strafing
        return (frontRightMotor.getCurrentPosition() + backLeftMotor.getCurrentPosition()) / 2.0;
    }

    public void resetStrafePosition() {
        lastStrafePosition = getStrafePosition();
        firstStrafeUpdate = true;
    }

    double targetDist;

    public CommandGroup straight(double dist, double maxPower) {
        return new SequentialGroup(
                new InstantCommand(() -> {
                    double startPos = getForwardPosition();
                    targetDist = startPos + dist;

                    drivePID.reset();
                    drivePID.setGoal(new KineticState(targetDist, 0, 0));
                }),

                new WaitUntil(() -> {
                    double currentPos = getForwardPosition();

                    double pidOutput = drivePID.calculate(new KineticState(currentPos));
                    pidOutput = Math.max(-maxPower, Math.min(maxPower, pidOutput));

                    runAllMotors(pidOutput, pidOutput, pidOutput, pidOutput);

                    double tolerance = 8.0;
                    return Math.abs(currentPos - targetDist) <= tolerance;
                }),

                new InstantCommand(() -> runAllMotors(0, 0, 0, 0))
        );
    }

    public CommandGroup strafeTo(double distance) {
        // Reset strafe PID
        strafePID.reset();
        resetStrafePosition();

        // Get current strafe position and calculate target
        double currentPos = getStrafePosition();
        double targetPos = currentPos + distance;

        // Set the goal for the strafe PID controller
        strafePID.setGoal(new KineticState(targetPos, 0, 0));

        return new SequentialGroup(
                new WaitUntil(() -> {
                    // Get current strafe position
                    double currentStrafePos = getStrafePosition();

                    // Calculate PID output based on current position
                    double pidOutput = strafePID.calculate(new KineticState(currentStrafePos));

                    // Apply strafe power to motors
                    // For strafing with mecanum wheels:
                    // - Strafing right: frontLeft/backRight forward, frontRight/backLeft backward
                    // - Strafing left: frontLeft/backRight backward, frontRight/backLeft forward
                    // Positive pidOutput = strafe right, Negative = strafe left

                    runAllMotors(
                            pidOutput,   // Front left: forward for right strafe
                            -pidOutput,  // Front right: backward for right strafe
                            -pidOutput,  // Back left: backward for right strafe
                            pidOutput    // Back right: forward for right strafe
                    );

                    // Check if we've reached the target within tolerance
                    double tolerance = 10.0; // encoder ticks tolerance
                    return Math.abs(currentStrafePos - targetPos) <= tolerance;
                }),
                new InstantCommand(() -> runAllMotors(0, 0, 0, 0))
        );
    }

    public CommandGroup strafeLeft(double distance) {
        // Positive distance = strafe right, so negative distance = strafe left
        return strafeTo(-distance);
    }

    public CommandGroup strafeRight(double distance) {
        return strafeTo(distance);
    }

    public CommandGroup turnTo(double targetHeadingDegrees2) {
        double targetHeadingDegrees = targetHeadingDegrees2 * dirMultiplier;
        return new SequentialGroup(
                new InstantCommand(() -> {
                    // Reset heading PID
                    headingPID.reset();

                    // Set the target heading
                    headingPID.setGoal(new KineticState(targetHeadingDegrees, 0, 0));
                }),
                new Delay(0.1),
                new WaitUntil(() -> {
                    // Get current heading
                    double currentHeading = getCurrentHeading();

                    // Calculate heading correction using PID
                    double headingCorrection = headingPID.calculate(new KineticState(currentHeading));

                    // Apply correction to motors for turning
                    runAllMotors(
                            -headingCorrection,  // Left motors go backward for right turn
                            headingCorrection,   // Right motors go forward for right turn
                            -headingCorrection,  // Left motors go backward for right turn
                            headingCorrection    // Right motors go forward for right turn
                    );

                    // Check if we're within tolerance (2 degrees by default)
                    double headingTolerance = 0.0; // degrees
                    double headingError = Math.abs(currentHeading - targetHeadingDegrees);
                    ActiveOpMode.telemetry().addData("error", headingError);

                    return Math.abs(headingCorrection) <= 0.15;
                }),
                new InstantCommand(() -> runAllMotors(0, 0, 0, 0))
        );
    }

    public CommandGroup turnBy(double angleDegrees) {
        // Get current heading and calculate absolute target
        double currentHeading = getCurrentHeading();
        double targetHeading = currentHeading + angleDegrees;

        // Normalize to 0-360 range
        targetHeading = normalizeAngle(targetHeading);

        return turnTo(targetHeading);
    }

    public double getCurrentHeading() {
        return continuousHeading;
    }

    public void updateContinuousHeading() {
        double current = imu.get().inDeg;

        if (firstHeadingUpdate) {
            lastImuAngle = current;
            continuousHeading = current;
            firstHeadingUpdate = false;
            return;
        }

        double delta = current - lastImuAngle;

        // Detect wrap crossing
        if (delta > 180) delta -= 360;
        if (delta < -180) delta += 360;

        continuousHeading += delta;
        lastImuAngle = current;
    }

    private double normalizeAngle(double angle) {
        // Normalize angle to 0-360 range
        angle %= 360;
        if (angle < 0) {
            angle += 360;
        }
        return angle;
    }

    public void resetHeading(double newHeading) {
        continuousHeading = newHeading;
        lastImuAngle = imu.get().inDeg;
    }

    public void loop() {
        updateContinuousHeading();
    }

    public CommandGroup getCalibGroup() {
        return new SequentialGroup(
                turnTo(100)
        );
    }

    public CommandGroup getTestGroup() {
        return new SequentialGroup(
                Shooter.INSTANCE.spinUp,
                Intake.INSTANCE.spinUp,
                straight(-2030, 0.4),
                new Delay(1),
                turnTo(Webcam.INSTANCE.imuTarget),
                Transfer.INSTANCE.opModeOverrideOn,
                new Delay(3),
                Transfer.INSTANCE.opModeOverrideOff,
                turnTo(33),
                straight(1550,0.6),
                new Delay(1),
                straight(-1550,0.6),
                turnTo(0),
                turnTo(Webcam.INSTANCE.imuTarget),
                Transfer.INSTANCE.opModeOverrideOn,
                new Delay(3),
                Transfer.INSTANCE.opModeOverrideOff,
                turnTo(90),
                straight(700,1),
                //strafeLeft(50),


                Shooter.INSTANCE.spinDown,
                Intake.INSTANCE.spinDown
                //turnTo(-54)
        );
    }

    public CommandGroup getFarzoneGroup() {
        return new SequentialGroup(
            Shooter.INSTANCE.spinUp,
                Intake.INSTANCE.spinUp,
                straight(200, 0.4),
                turnTo(22),
                new Delay(2),
                //turnTo(Webcam.INSTANCE.imuTarget),
                Transfer.INSTANCE.opModeOverrideOn,
                new Delay(5),
                Transfer.INSTANCE.opModeOverrideOff,
                turnTo(88),
                straight(1450, 0.5),
                new Delay(5),
                straight(-1500, 0.5),
                turnTo(40),
                new Delay(3),
                turnTo(Webcam.INSTANCE.imuTarget),
                Transfer.INSTANCE.opModeOverrideOn,
                new Delay(5),
                Transfer.INSTANCE.opModeOverrideOff,
                Shooter.INSTANCE.spinDown,
                Intake.INSTANCE.spinDown,
                straight(700, 1)
        );
    }

    public CommandGroup getGroup2() {
        return new SequentialGroup(

                //turnTo(-54)
        );
    }

    public CommandGroup getAutoGroup() {
        return new SequentialGroup(
                Shooter.INSTANCE.spinUp,
                Intake.INSTANCE.spinUp,
                straight(-100, 0.6),
                Shooter.INSTANCE.waitForSpeed,
                // shoot preload
                Transfer.INSTANCE.opModeOverrideOn,
                new Delay(3),
                Transfer.INSTANCE.opModeOverrideOff,

                // 1st line intake
                turnTo(-55),
                straight(100, 0.6),
                straight(-100, 0.6),

                // 1st line shoot
                turnTo(0),
                Transfer.INSTANCE.opModeOverrideOn,
                new Delay(3),
                Transfer.INSTANCE.opModeOverrideOff,

                // 2nd line intake
                turnTo(-52),
                strafeLeft(50),
                straight(120, 0.6),
                straight(-120, 0.6),
                strafeRight(-50),

                // 2nd line shoot
                turnTo(0),
                Transfer.INSTANCE.opModeOverrideOn,
                new Delay(3),
                Transfer.INSTANCE.opModeOverrideOff,

                // park
                Shooter.INSTANCE.spinDown,
                Intake.INSTANCE.spinDown,
                strafeLeft(50)
        );
    }
}