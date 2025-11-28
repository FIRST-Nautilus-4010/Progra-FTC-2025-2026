package org.firstinspires.ftc.teamcode.Test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name="TuneShooterVelocity", group="test")
@Config
public class TuneShooterVelocity extends OpMode {
    private DcMotorEx launcherTop;
    private DcMotorEx launcherBottom;

    public static double vel = -1400;
    public static double openLoopPower = 0.75;
    private Servo hammerShooter;

    public static double kP = 3.7;
    public static double kI = 16383.5000077;
    public static double kD = 0;
    public static double kF = 0;

    Telemetry dashboardTelemetry;

    @Override
    public void init() {
        launcherTop = hardwareMap.get(DcMotorEx.class, "launcherTop");
        launcherBottom = hardwareMap.get(DcMotorEx.class, "launcherBottom");
        hammerShooter = hardwareMap.get(Servo.class, "hammerS");
        FtcDashboard dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();
        launcherBottom.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        launcherTop.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(kP, kI, kD, kF));
        launcherBottom.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(kP, kI, kD, kF));

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


        dashboardTelemetry.addData("Velocity in tps launcher top", launcherTop.getVelocity());
        dashboardTelemetry.addData("Velocity in tps launcher bottom", launcherBottom.getVelocity());
        dashboardTelemetry.update();
    }
}