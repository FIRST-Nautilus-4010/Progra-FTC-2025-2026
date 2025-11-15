package org.firstinspires.ftc.teamcode.Subsystems.Vision;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;


public class PoseUtils {

  /*
     Convierte yaw, pitch, roll a un quaternion

    public static Quaternion eulerToQuaternion(double roll, double pitch, double yaw) {
        return Rotation3d.fromEulerXYZ(roll, pitch, yaw).getQuaternion();
    }


    public static Pose3D makePose(Vector3d pos, Rotation3d rot) {
        return new Pose3D(pos, rot.getQuaternion());
    }

    public static Pose3D applyCameraOffset(Pose3D cameraPose, Vector3d camPos, Rotation3d camRot) {
        // La pose del robot = pose global de cámara transformada por el opuesto del offset
        Pose3D offset = new Pose3D(camPos, camRot.getQuaternion());
        return cameraPose.minus(offset);
    }
    */
}

