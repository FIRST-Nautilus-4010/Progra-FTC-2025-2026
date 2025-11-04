package org.firstinspires.ftc.teamcode.Subsystems.Intake.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.Subsystems.Intake.IntakeIO;

public class Take implements Action {
    private final IntakeIO io;
    private int lastUsedStage = 0;

    public Take(IntakeIO io) {
        this.io = io;
    }


    @Override
    public boolean run(@NonNull TelemetryPacket packet) {

        io.setPwr(0.3);

        if (isFinished()) {
            onEnd();
        }

        return !isFinished();
    }

    public void onEnd() {
        io.setVel(0);
    }

    public boolean isFinished() {
        return false;
    }
}
