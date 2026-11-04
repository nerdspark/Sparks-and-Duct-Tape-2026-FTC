package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterMetadata;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

public class ClusterData {
    public int percentFound;
    public Pose3D pose;
    public AprilTagClusterMetadata metadata;

    public ClusterData(int percentClusterFound, Pose3D pose, AprilTagClusterMetadata data) {
        this.percentFound = percentClusterFound;
        this.pose = pose;
        this.metadata = data;
    }
}
