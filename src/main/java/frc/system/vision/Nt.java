package frc.system.vision;

import java.util.EnumSet;
import java.util.Optional;
import java.util.function.Consumer;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.system.Vision;

public class Nt implements Vision {
	final Measurement cached = new Measurement();
	final AprilTagFieldLayout tags = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

	@Override
	public void register(Consumer<Measurement> measurement) {
		NetworkTableInstance nt = NetworkTableInstance.getDefault();

		nt.addListener(nt.getTopic("/AprilTag/Detections"), EnumSet.of(NetworkTableEvent.Kind.kValueAll), (event) -> {
			double[] data = event.valueData.value.getDoubleArray();

			if (data.length % 8 != 0)
				System.err.println("WARN: Invalid NetworkTables AprilTag vision data array " + data.length);

			int n = 0;
			double pX = 0, pY = 0, pZ = 0, rX = 0, rY = 0, rZ = 0;

			for (int i = 0; i + 8 < data.length; i += 8) {
				int tagId = (int) data[i];

				double x = data[i + 1], y = data[i + 2], z = data[i + 3];

				Translation3d translation = new Translation3d(x, y, z);

				double qw = data[i + 4], qx = data[i + 5], qy = data[i + 6], qz = data[i + 7];

				Rotation3d rotation = new Rotation3d(new Quaternion(qw, qx, qy, qz));

				Pose3d pose = new Pose3d(translation, rotation);

				Optional<Pose3d> maybeTagPose = tags.getTagPose(tagId);

				if (maybeTagPose.isEmpty()) {
					System.err.println("WARN: Unknown tag id " + tagId + "detected");
					break;
				}

				Pose3d tagPose = maybeTagPose.get();
				Pose3d tagFromCamera = tagPose.relativeTo(pose);

				// TODO: get camera pose relative to robot

				n++;

				pX += tagFromCamera.getX();
				pY += tagFromCamera.getY();
				pZ += tagFromCamera.getZ();

				rX += tagFromCamera.getRotation().getX();
				rY += tagFromCamera.getRotation().getY();
				rZ += tagFromCamera.getRotation().getZ();
			}

			cached.pose = new Pose3d(pX / n, pY / n, pZ / n, new Rotation3d(rX / n, rY / n, rZ / n));
			cached.timestamp = event.valueData.value.getTime();
			// TODO: calculate standard deviations of pose measurement.
			cached.stdDev = null;
		});
	}
}
