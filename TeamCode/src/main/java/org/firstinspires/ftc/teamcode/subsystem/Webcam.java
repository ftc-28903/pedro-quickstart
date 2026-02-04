package org.firstinspires.ftc.teamcode.subsystem;

import static dev.nextftc.ftc.ActiveOpMode.isStopRequested;

import android.util.Size;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.concurrent.TimeUnit;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.Direction;
import dev.nextftc.hardware.impl.IMUEx;

@Configurable
public class Webcam implements Subsystem {
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;
    private List<AprilTagDetection> detectedTags = new ArrayList<>();
    // goal auto approx, far should see it (almost) immediately
    public DistanceComponents lastDistanceComponent = new DistanceComponents(120,45);
    private final IMUEx imu = new IMUEx("imu", Direction.UP, Direction.FORWARD);

    private static final double CAMERA_TILT_DEGREES = 15.0; // Camera tilted upwards
    public static float decimation = 2.0f;
    private TelemetryManager telemetryM;
    public double imuTarget = 0;
    public double lastOffset = 0;
    public double imuOffset = 0;

    // abs heading
    private double continuousHeading = 0;
    private double lastImuAngle = 0;
    private boolean firstHeadingUpdate = true;

    /**
     * Calculates the horizontal distance to an AprilTag, excluding height difference.
     * Takes into account the camera's upward tilt.
     *
     * @param detection The AprilTag detection
     * @return Horizontal distance in cm, or -1 if detection is null
     */
    public double getHorizontalDistance(AprilTagDetection detection) {
        if (detection == null) return -1;

        // Get the range (direct distance) and elevation angle from the detection
        double range = detection.ftcPose.range; // in cm
        double elevationAngle = detection.ftcPose.elevation; // in degrees

        // Adjust elevation angle for camera tilt
        double actualElevationAngle = elevationAngle + CAMERA_TILT_DEGREES;

        // Calculate horizontal distance using cosine
        // horizontalDistance = range * cos(actualElevation)
        double horizontalDistance = range * Math.cos(Math.toRadians(actualElevationAngle));

        return horizontalDistance;
    }

    /**
     * Alternative method: Gets the horizontal distance and height difference separately
     */
    public DistanceComponents getDistanceComponents(AprilTagDetection detection) {
        if (detection == null) return null;

        double range = detection.ftcPose.range;
        double elevationAngle = detection.ftcPose.elevation;
        double actualElevationAngle = elevationAngle + CAMERA_TILT_DEGREES;

        double horizontalDistance = range * Math.cos(Math.toRadians(actualElevationAngle));
        double heightDifference = range * Math.sin(Math.toRadians(actualElevationAngle));

        return new DistanceComponents(horizontalDistance-30, heightDifference);
    }

    // Helper class to return both components
    public static class DistanceComponents {
        public final double horizontal; // cm
        public final double vertical;   // cm

        public DistanceComponents(double horizontal, double vertical) {
            this.horizontal = horizontal;
            this.vertical = vertical;
        }
    }

    public double getTurnToBackOfTag(AprilTagDetection detection) {
        if (detection == null) return 0;

        DistanceComponents d = getDistanceComponents(detection);
        double forward = d.horizontal;

        double bearingRad = Math.toRadians(detection.ftcPose.bearing);
        double side = forward * Math.tan(bearingRad);

        double x_tag = side;
        double y_tag = forward;

        double tagYawRad = Math.toRadians(detection.ftcPose.yaw);

        double backX = 0 * Math.sin(tagYawRad);
        double backY = 0 * Math.cos(tagYawRad);

        double targetX = x_tag + backX;
        double targetY = y_tag + backY;

        return Math.toDegrees(Math.atan2(targetX, targetY));
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

    public void init() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .build();

        aprilTagProcessor.setDecimation(2.0f);

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(ActiveOpMode.hardwareMap().get(WebcamName.class, "Webcam 1"));
        builder.setCameraResolution(new Size(640, 480));
        builder.addProcessor(aprilTagProcessor);

        visionPortal = builder.build();

        setManualExposure(2, 100);
    }

    @Override
    public void periodic() {
        updateContinuousHeading();
        aprilTagProcessor.setDecimation(decimation);
        detectedTags = aprilTagProcessor.getDetections();

        AprilTagDetection allianceTag = getTagBySpecificId(20);
        if(allianceTag != null && allianceTag.metadata != null) {
            lastDistanceComponent = getDistanceComponents(allianceTag);
            lastOffset = getTurnToBackOfTag(allianceTag);
            imuTarget = continuousHeading - lastOffset;
        }

        displayDetectionTelemetry(allianceTag);
        
        StringBuilder sb = new StringBuilder("detected tags: ");
        for (AprilTagDetection detection : getDetectedTags()) {
            sb.append(detection.id);
        }
        ActiveOpMode.telemetry().addLine(sb.toString());
        telemetryM.addData("goalLastOffset", lastOffset);
        telemetryM.addData("goalIMUTarget", imuTarget);
        imuOffset = continuousHeading - imuTarget;
        telemetryM.addData("goalIMUOffset", imuOffset);
    }

    public List<AprilTagDetection> getDetectedTags() {
        return detectedTags;
    }

    public void displayDetectionTelemetry(AprilTagDetection detectedId) {
        if (detectedId == null) return;

        if (detectedId.metadata != null) {
            ActiveOpMode.telemetry().addLine(String.format(Locale.ENGLISH, "\n==== (ID %d) %s", detectedId.id, detectedId.metadata.name));
            ActiveOpMode.telemetry().addLine(String.format(Locale.ENGLISH, "XYZ %6.1f %6.1f %6.1f (cm)", detectedId.ftcPose.x, detectedId.ftcPose.y, detectedId.ftcPose.z));
            ActiveOpMode.telemetry().addLine(String.format(Locale.ENGLISH, "PRY %6.1f %6.1f %6.1f (deg)", detectedId.ftcPose.pitch, detectedId.ftcPose.roll, detectedId.ftcPose.yaw));
            ActiveOpMode.telemetry().addLine(String.format(Locale.ENGLISH, "RBE %6.1f %6.1f %6.1f (cm, deg, deg)", detectedId.ftcPose.range, detectedId.ftcPose.bearing, detectedId.ftcPose.elevation));
            ActiveOpMode.telemetry().addLine(String.format(Locale.ENGLISH, "Horizontal: %6.1f, vertical: %6.1f", lastDistanceComponent.horizontal, lastDistanceComponent.vertical));
        } else {
            ActiveOpMode.telemetry().addLine(String.format(Locale.ENGLISH, "\n==== (ID %d) Unknown", detectedId.id));
            ActiveOpMode.telemetry().addLine(String.format(Locale.ENGLISH, "Center %6.0f %6.0f (pixels)", detectedId.center.x, detectedId.center.y));
        }
    }

    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    private boolean setManualExposure(int exposureMS, int gain) {
        // Ensure Vision Portal has been setup.
        if (visionPortal == null) {
            return false;
        }

        // Wait for the camera to be open
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            ActiveOpMode.telemetry().addData("Camera", "Waiting");
            ActiveOpMode.telemetry().update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            ActiveOpMode.telemetry().addData("Camera", "Ready");
            ActiveOpMode.telemetry().update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            // Set exposure.  Make sure we are in Manual Mode for these values to take effect.
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure(exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);

            // Set Gain.
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
            return true;
        } else {
            return false;
        }
    }

    public AprilTagDetection getTagBySpecificId(int id) {
        for (AprilTagDetection detection : detectedTags) {
            if (detection.id == id) {
                return detection;
            }
        }
        return null;
    }

    public static final Webcam INSTANCE = new Webcam();
    private Webcam() { }
}
