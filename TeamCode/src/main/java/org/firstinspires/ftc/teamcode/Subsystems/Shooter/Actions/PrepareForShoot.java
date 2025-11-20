package org.firstinspires.ftc.teamcode.Subsystems.Shooter.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.ShooterIO;

import java.util.function.Supplier;

public class PrepareForShoot implements Action {
    private final ShooterIO io;

    private final Supplier<Double> distanceWithTargetX;
    private final Supplier<Double> distanceWithTargetY;
    private final Supplier<Double> botYaw;

    private final double targetHeight = 0.7;
    private double accel = -9.81;
    private final Telemetry telemetry;

    private double pitch;
    private double yaw;
    private double vel;



    public PrepareForShoot(ShooterIO io, Supplier<Double> distanceWithTargetX, Supplier<Double> distanceWithTargetY, Supplier<Double> botYaw, Telemetry telemetry) {
        this.io = io;
        this.distanceWithTargetX = distanceWithTargetX;
        this.distanceWithTargetY = distanceWithTargetY;
        this.botYaw = botYaw;

        this.telemetry = telemetry;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        double distance = Math.hypot(distanceWithTargetY.get() * 0.0254, distanceWithTargetX.get() * 0.0254);

        vel = 800;

        double a = -(accel * distance * distance) / (2 * 4.272 * 4.272);
        double b = distance;
        double c = a - targetHeight;

        double x0 = Math.toRadians(90) - Math.atan((-b+Math.sqrt(b*b - 4*a*c))/2*a);
        double x1 = Math.toRadians(90) - Math.atan((-b-Math.sqrt(b*b - 4*a*c))/2*a);

        if (x0 < 0 ) {
            pitch = x1;
        } else {
            pitch = x0;
        }
        yaw = Math.atan2(distanceWithTargetY.get() * 0.0254, distanceWithTargetX.get() * 0.0254) - botYaw.get();

        io.setYaw(yaw);
        io.setPitch(pitch);
        io.setVel(vel);

        //io.setPitch(Math.toRadians(45));
        //io.setPitch(1);

        telemetry.addData("desiredShooterPitch", pitch);
        telemetry.addData("desiredShooterYaw", yaw);
        telemetry.addData("desiredShooterVel", vel);
        telemetry.addData("desiredShooterX", distanceWithTargetX.get() * 0.0254);
        telemetry.addData("desiredShooterY", distanceWithTargetY.get() * 0.0254);



        return !isFinished();
    }

    public boolean isFinished() {
        return Math.abs(io.getVel() - vel) < 50 && Math.abs(io.getYaw() - yaw) < 0.1;
    }
}
