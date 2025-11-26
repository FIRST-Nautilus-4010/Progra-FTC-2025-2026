package org.firstinspires.ftc.teamcode.Subsystems.Shooter.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Intake.IntakeIO;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.ShooterIO;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

import java.util.function.Supplier;

public class PrepareForShoot implements Action {
    private final ShooterIO io;
    private final IntakeIO intakeIO;

    private final Supplier<Double> distanceWithTargetX;
    private final Supplier<Double> distanceWithTargetY;
    private final Supplier<Double> botYaw;

    private final double targetHeight = 0.5;
    private double accel = 2.5;
    private final Telemetry telemetry;

    private ElapsedTime elapsedTime;
    private boolean initialized  = false;

    private double pitch;
    private double yaw;
    private double vel;
    private double distance;

    private final Supplier<AprilTagDetection> tagDetection;



    public PrepareForShoot(ShooterIO io, Supplier<Double> distanceWithTargetX, Supplier<Double> distanceWithTargetY, Supplier<Double> botYaw, Supplier<AprilTagDetection> tagDetection, double accel, Telemetry telemetry) {
        this.io = io;
        this.intakeIO = new IntakeIO(io.getHardwareMap());
        this.distanceWithTargetX = distanceWithTargetX;
        this.distanceWithTargetY = distanceWithTargetY;
        this.botYaw = botYaw;

        this.telemetry = telemetry;
        this.tagDetection = tagDetection;

        this.accel = accel;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {

        if (!initialized) {
            elapsedTime = new ElapsedTime();
            initialized = true;
        }

        if (tagDetection.get() == null) {
            // ---- CÁLCULO DEL YAW---
             yaw = Math.atan2(distanceWithTargetY.get() * 0.0254,
                    distanceWithTargetX.get() * 0.0254)
                    - botYaw.get();
            distance = Math.hypot(distanceWithTargetY.get() * 0.0254,
                    distanceWithTargetX.get() * 0.0254);

        } else  {
            AprilTagPoseFtc pose = tagDetection.get().ftcPose;

            yaw = Math.toRadians(pose.yaw);
            distance = Math.hypot(pose.x, pose.y) * 0.0254;
        }

        double vel = 7;

        if (distance < 2) {
            vel = 5.4;
        }



        double g = accel;
        double x = distance;
        double y = targetHeight;

        double A = (g * x * x) / (2.0 * vel * vel);
        double B = -x;
        double C = y - A;

        double discriminant = B*B - 4*A*C;

        double pitch;

        if (discriminant < 0) {
            pitch = Math.toRadians(45);  // velocidad insuficiente
        } else {
            double sqrtD = Math.sqrt(discriminant);

            double T1 = (-B + sqrtD) / (2*A);
            double T2 = (-B - sqrtD) / (2*A);

            // convertimos T = tan(theta) → theta
            double theta1 = Math.atan(T1);
            double theta2 = Math.atan(T2);

            // elige ángulo válido
            if (theta1 > Math.toRadians(10) && theta1 < Math.toRadians(80)) {
                pitch = theta1;
            } else if (theta2 > Math.toRadians(10) && theta2 < Math.toRadians(80)) {
                pitch = theta2;
            } else {
                pitch = Math.toRadians(45);
            }
        }

        io.setYaw(yaw);
        io.setPitch(pitch);
        //io.setPitch(Math.toRadians(45));
        io.setVel(((vel + velOffset) * 60) / (0.1016 * Math.PI));

        telemetry.addData("desiredShooterPitch", pitch);
        telemetry.addData("desiredShooterYaw", yaw);
        telemetry.addData("desiredShooterVel", (vel * 60) / (0.1016 * Math.PI));
        telemetry.addData("desiredShooterX", distanceWithTargetX.get() * 0.0254);
        telemetry.addData("desiredShooterY", distanceWithTargetY.get() * 0.0254);

        return !isFinished();
    }

    public boolean isFinished() {
        return elapsedTime.milliseconds() > 2000;
    }
}
