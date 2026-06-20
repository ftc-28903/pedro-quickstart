package org.firstinspires.ftc.teamcode.subsystem;

import static dev.nextftc.ftc.ActiveOpMode.isStopRequested;

import android.util.Size;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utils.AutoStorage;
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

    private final IMUEx imu = new IMUEx("imu", Direction.UP, Direction.FORWARD);
    public static float decimation = 2.0f;
    private TelemetryManager telemetryM;

    public ElapsedTime detectionTimer = new ElapsedTime();
    private double continuousHeading = 0;
    private double lastImuAngle = 0;
    private boolean firstHeadingUpdate = true;

    // Camera mounting pitch configuration (30 degrees tilted upwards)
    private static final double CAMERA_PITCH_RAD = Math.toRadians(30.0);

    // --- WEBCAM OFFSETS (Inches) ---
    // Physical distance from the ROBOT CENTER to the CAMERA LENS measured in the Robot Frame.
    // Pedro Pathing Frame Convention: +X is Forward, +Y is Left
    private static final double CAMERA_OFFSET_X = 4.7;  // Forward/Backward placement shift
    private static final double CAMERA_OFFSET_Y = 6.3;  // Side-to-side placement shift

    // Estimated Robot Pose via AprilTag (Stored in Native Inches to match Pedro Pathing)
    private RobotPose estimatedPose = new RobotPose(0, 0, 0);

    public static class RobotPose {
        public double x, y, heading;
        public RobotPose(double x, double y, double heading) {
            this.x = x;
            this.y = y;
            this.heading = heading;
        }
    }

    /**
     * Estimates the absolute robot position using Pedro Pathing map coordinates.
     * Accounts for a 90-degree camera yaw rotation, a 30-degree upward camera tilt, and mounting offsets.
     */
    public RobotPose estimateRobotPose(AprilTagDetection detection) {
        if (detection == null || detection.metadata == null) return null;

        // 1. Get known static global field coordinates for this tag ID (in inches)
        double tagFieldX = getTagFieldX(detection.id);
        double tagFieldY = getTagFieldY(detection.id);

        // 2. Fetch raw camera relative translation metrics (convert cm to inches)
        double rawCamX = detection.ftcPose.x / 2.54; // Camera side-to-side
        double rawCamY = detection.ftcPose.y / 2.54; // Camera straight forward out lens
        double rawCamZ = detection.ftcPose.z / 2.54; // Camera height/vertical

        // 3. STEP A: Counter-rotate the 30° upward camera tilt pitch around the local camera X axis
        // This flattens out raw forward (Y) and vertical (Z) distances to the ground plane
        double flatCamX = rawCamX;
        double flatCamY = rawCamY * Math.cos(CAMERA_PITCH_RAD) - rawCamZ * Math.sin(CAMERA_PITCH_RAD);

        // 3. STEP B: Account for the 90-degree physical mounting rotation of the camera body
        // Assuming camera faces RIGHT relative to the chassis frame:
        // Robot Frame X (Forward) = -flatCamY (moving deeper into camera view means robot is moving left/right)
        // Robot Frame Y (Left)    = -flatCamX
        // NOTE: Invert these signs if your tracking values shift backward or if camera faces LEFT!
        double robotRelativeX = -flatCamY;
        double robotRelativeY = -flatCamX;

        // 4. Transform local camera-to-target vectors into global Pedro Pathing coordinate alignment
        double globalHeadingRad = Math.toRadians(continuousHeading);

        // Calculate global position of the CAMERA lens
        double cameraFieldX = tagFieldX - (robotRelativeX * Math.cos(globalHeadingRad) - robotRelativeY * Math.sin(globalHeadingRad));
        double cameraFieldY = tagFieldY - (robotRelativeX * Math.sin(globalHeadingRad) + robotRelativeY * Math.cos(globalHeadingRad));

        // 5. Translate from the camera lens back to the robot center using the robot-frame camera offsets
        double fieldX = cameraFieldX - (CAMERA_OFFSET_X * Math.cos(globalHeadingRad) - CAMERA_OFFSET_Y * Math.sin(globalHeadingRad));
        double fieldY = cameraFieldY - (CAMERA_OFFSET_X * Math.sin(globalHeadingRad) + CAMERA_OFFSET_Y * Math.cos(globalHeadingRad));

        return new RobotPose(fieldX, fieldY, continuousHeading);
    }

    /**
     * Absolute tag X positions on the field map (Inches)
     */
    private double getTagFieldX(int id) {
        switch (id) {
            case 24:
                return 72.0 + 55.6; // 127.6 inches (Left side)
            case 20: // blue
                return 72.0 - 55.6; // 16.4 inches (Right side)
            default:
                return 0.0;
        }
    }

    /**
     * Absolute tag Y positions on the field map (Inches)
     */
    private double getTagFieldY(int id) {
        return 72.0 + 58.3;
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

        aprilTagProcessor.setDecimation(decimation);

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(ActiveOpMode.hardwareMap().get(WebcamName.class, "Webcam 1"));
        builder.setCameraResolution(new Size(640, 480));
        builder.addProcessor(aprilTagProcessor);

        visionPortal = builder.build();

        setManualExposure(0, 100);
    }

    @Override
    public void periodic() {
        if (!AutoStorage.opModeStarted || AutoStorage.follower == null || aprilTagProcessor == null) return;
        continuousHeading = Math.toDegrees(AutoStorage.follower.getHeading());
        detectedTags = aprilTagProcessor.getDetections();

        double sumX = 0;
        double sumY = 0;
        int validDetectionsCount = 0;

        // Iterate through all active detections, filtering explicitly for IDs 20 & 24
        for (AprilTagDetection detection : detectedTags) {
            if (detection != null && detection.metadata != null && (detection.id == 20 || detection.id == 24)) {
                RobotPose calculation = estimateRobotPose(detection);
                if (calculation != null) {
                    sumX += calculation.x;
                    sumY += calculation.y;
                    validDetectionsCount++;
                }
            }
        }

        // If one or both targets are tracked, update the shared tracking estimation
        if (validDetectionsCount > 0) {
            estimatedPose = new RobotPose(sumX / validDetectionsCount, sumY / validDetectionsCount, continuousHeading);
            detectionTimer.reset();
        }

        // Provide telemetry output stream layout tracking metrics
        StringBuilder sb = new StringBuilder("tracked tags: ");
        for (AprilTagDetection d : detectedTags) {
            if (d.id == 20 || d.id == 24) sb.append(d.id).append(" ");
        }
        ActiveOpMode.telemetry().addLine(sb.toString());

        telemetryM.addData("WEBCAMMMMM Robot Global X (in)", estimatedPose.x);
        telemetryM.addData("Robot Global Y (in)", estimatedPose.y);
        telemetryM.addData("Robot Global Heading", estimatedPose.heading);
    }

    public List<AprilTagDetection> getDetectedTags() {
        return detectedTags;
    }

    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    private boolean setManualExposure(int exposureMS, int gain) {
        if (visionPortal == null) {
            return false;
        }

        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            ActiveOpMode.telemetry().addData("Camera", "Waiting");
            ActiveOpMode.telemetry().update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            ActiveOpMode.telemetry().addData("Camera", "Ready");
            ActiveOpMode.telemetry().update();
        }

        if (!isStopRequested()) {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure(exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);

            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
            return true;
        } else {
            return false;
        }
    }

    public RobotPose getLatestEstimatedPose() {
        return this.estimatedPose;
    }

    public static final Webcam INSTANCE = new Webcam();
    private Webcam() { }
}