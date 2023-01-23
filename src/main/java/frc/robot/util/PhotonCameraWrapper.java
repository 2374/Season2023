package frc.robot.util;

import java.util.ArrayList;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.Constants;

public class PhotonCameraWrapper {
    public PhotonCamera camera;
    public PhotonPoseEstimator estimator;

    public PhotonCameraWrapper() {
        final AprilTag tag1 = new AprilTag(1,
                new Pose3d(Constants.FIELD_LENGTH, Constants.FIELD_WIDTH, 0, new Rotation3d(0, 0, 180)));
        ArrayList<AprilTag> tagList = new ArrayList<>();
        tagList.add(tag1);
        AprilTagFieldLayout fieldLayout = new AprilTagFieldLayout(tagList, Constants.FIELD_LENGTH,
                Constants.FIELD_WIDTH);
        camera = new PhotonCamera(Constants.CAMERA_NAME);
        estimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camera,
                Constants.ROBOT_TO_CAMERA);
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        estimator.setReferencePose(prevEstimatedRobotPose);
        return estimator.update();
    }
}
