package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.util.ClusterData;
import org.firstinspires.ftc.teamcode.util.Constants.Shoot;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;

public class ApriltagManager {
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal portal;
    private String alliance;
    private ClusterData latestData = null;
    private FtcDashboard dashboard;

    public ApriltagManager(HardwareMap hardwareMap, String alliance, FtcDashboard dash) {
        this.alliance = alliance;
        this.dashboard = dash;

        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setDrawCubeProjection(false)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(false)
                .setSuppressCalibrationWarnings(false)
                .setLensIntrinsics(867.316568133, 867.316568133, 668.966517635, 388.591225738)
                .setNumThreads(2) // Has 4 total
//                .setCameraPose(
//                        new Position(DistanceUnit.METER, 0, -0.092794, 0.096738, 0),
//                        new YawPitchRollAngles(AngleUnit.DEGREES, 0, -35, 0, 0)
//                )
                .setOutputUnits(DistanceUnit.METER, AngleUnit.DEGREES)
                .build();

        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "webcam"))
                .addProcessor(aprilTagProcessor)
                .setCameraResolution(new Size(1280, 800))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();
        portal.setProcessorEnabled(aprilTagProcessor, true);

        dashboard.startCameraStream(portal, 30);
    }

    public void getClusters() {
        TelemetryPacket packet = new TelemetryPacket();
        ArrayList<AprilTagDetection> detections = aprilTagProcessor.getFreshDetections();

        if (detections == null) {
            return;
        }

        for (AprilTagDetection detection : detections) {
            if (detection instanceof AprilTagClusterDetection) {
                AprilTagClusterDetection clusterDetection = (AprilTagClusterDetection) detection;
                if (clusterDetection.metadata.name.toLowerCase().startsWith(alliance)) {
                    if (Math.abs(clusterDetection.ftcPose.roll) < 90) {
                        packet.put("Robot X", clusterDetection.ftcPose.x);
                        packet.put("Robot Y", clusterDetection.ftcPose.y);
                        packet.put("Robot Z", clusterDetection.ftcPose.z);
                        packet.put("Robot Yaw", clusterDetection.ftcPose.yaw);
                        dashboard.sendTelemetryPacket(packet);

                        double cameraTilt = Math.toRadians(55.0);
                        double cosTilt = Math.cos(cameraTilt);
                        double sinTilt = Math.sin(cameraTilt);

                        double robotY = clusterDetection.ftcPose.y * cosTilt - clusterDetection.ftcPose.z * sinTilt - 0.092794;
                        double robotZ = clusterDetection.ftcPose.y * sinTilt + clusterDetection.ftcPose.z * cosTilt + 0.096738;

                        double pitchPrime = Math.atan2(robotZ, robotY);

                        double yT = robotY + Shoot.L * Math.cos(pitchPrime);
                        double zT = robotZ - Shoot.L * Math.sin(pitchPrime);

                        double bearing = Math.toDegrees(Math.atan2(-clusterDetection.ftcPose.x, yT));

                        Pose3D pose = new Pose3D(
                                new Position(DistanceUnit.METER, clusterDetection.ftcPose.x, yT, zT, 0),
                                new YawPitchRollAngles(AngleUnit.DEGREES, bearing, pitchPrime, 0, 0)
                        );

                        latestData = new ClusterData(clusterDetection.percentClusterFound, pose, clusterDetection.metadata);
                    }
                }
            }
        }
    }

    public ClusterData getLatestData() {
        return latestData;
    }
}