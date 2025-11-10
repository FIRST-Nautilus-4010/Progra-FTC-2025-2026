package org.firstinspires.ftc.teamcode.Subsystems.Intake;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Intake.Actions.Shoot;
import org.firstinspires.ftc.teamcode.Subsystems.Intake.Actions.Take;
import org.firstinspires.ftc.teamcode.Subsystems.Intake.Actions.setVel;
import org.firstinspires.ftc.teamcode.Subsystems.Intake.IntakeIO;

public class IntakeSubsystem {
    private final IntakeIO io;
    private final Telemetry telemetry;

    public IntakeSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        io = new IntakeIO(hardwareMap);

        this.telemetry = telemetry;
    }

    public Action take() {
        return new Take(io);
    }

    public Action shoot() {
        return new Shoot(io);
    }

    public Action stop() {
        return new setVel(io, 0);
    }


    // TODO: agregar acción que detenga el intake al entrar al estado travel
    public Action travel() {
        return null;
    }
    public void periodic() {
        telemetry.addData("Stage 1 state", io.onStage1());
        telemetry.addData("Stage 2 state", io.onStage2());
        telemetry.addData("Stage 3 state", io.onStage3());

        telemetry.addData("Intake vel", io.getVel());

    }
}
