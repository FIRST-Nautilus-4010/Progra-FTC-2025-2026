package org.firstinspires.ftc.teamcode.Subsystems.Intake.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.Subsystems.Intake.IntakeIO;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.ShooterIO;

public class setVel implements Action {
    private final IntakeIO io;

    private final double vel;

    public setVel(IntakeIO io, double vel) {
        this.io = io;
        this.vel = vel;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        io.setVel(vel);
        return !isFinished();
    }

    public boolean isFinished() {
        return true;
    }
}
