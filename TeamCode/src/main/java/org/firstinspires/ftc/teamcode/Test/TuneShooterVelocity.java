package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp(name="TuneShooterVelocity", group="test")
public class TuneShooterVelocity extends OpMode {
    private DcMotorEx launcherTop;
    private DcMotorEx launcherBottom;

    public static double vel = 1700;
    public static double openLoopPower = 0.75;

    public static double kP = 0;
    public static double kI = 0;
    public static double kF = 0;

    @Override
    public void init() {
        launcherTop = hardwareMap.get(DcMotorEx.class, "launcherTop");
        launcherBottom = hardwareMap.get(DcMotorEx.class, "launcherBottom");

        launcherBottom.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        launcherTop.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(kP, kI, 0, kF));
        launcherBottom.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(kP, kI, 0, kF));

        // A → velocity mode
        if (gamepad1.a) {
            launcherTop.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            launcherBottom.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            launcherTop.setVelocity(vel);
            launcherBottom.setVelocity(vel);
        }

        // B → open loop
        if (gamepad1.b) {
            launcherTop.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            launcherBottom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            launcherTop.setPower(openLoopPower);
            launcherBottom.setPower(openLoopPower);
        }

        telemetry.addData("Velocity in tps launcher top", launcherTop.getVelocity());
        telemetry.addData("Velocity in tps launcher bottom", launcherBottom.getVelocity());
        telemetry.update();
    }
}
