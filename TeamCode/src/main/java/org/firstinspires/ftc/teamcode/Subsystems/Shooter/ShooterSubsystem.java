package org.firstinspires.ftc.teamcode.Subsystems.Shooter;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Subsystems.Shooter.Actions.PrepareForShoot;

public class ShooterSubsystem {
    private final ShooterIO io;

    public ShooterSubsystem(HardwareMap hardwareMap) {
        io = new ShooterIO(hardwareMap);
    }

    public Action prepareForShoot() {
        return new PrepareForShoot(io, () -> 1.5, () -> 1.5);
    }

    public void intake() {
        io.setVel(-6000);
    }

    public void stop() {
        io.setVel(0);
    }

}