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
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.ShooterIO;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.VisionIO;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name="TuneHood", group="test")
@Config
public class TuneHood extends OpMode {

    private Servo hood;
    private DcMotorEx launcherTop;
    private DcMotorEx launcherBottom;
    private Servo hammerShooter;

    public static double servoPos;
    Telemetry dashboardTelemetry;


    public static final PIDFController shooterController = new PIDFController(TuneShooterVelocity.shooterCoeffs);

    @Override
    public void init() {
        hood = hardwareMap.get(Servo.class, "shooterPitch");

        launcherTop = hardwareMap.get(DcMotorEx.class, "launcherTop");
        launcherBottom = hardwareMap.get(DcMotorEx.class, "launcherBottom");

        launcherBottom.setDirection(DcMotorSimple.Direction.REVERSE);

        launcherTop.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launcherBottom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        hammerShooter = hardwareMap.get(Servo.class, "hammerS");
        FtcDashboard dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();

    }

    @Override
    public void loop() {

        shooterController.setCoefficients(TuneShooterVelocity.shooterCoeffs);

        shooterController.setTolerance(20);
        shooterController.setSetPoint(TuneShooterVelocity.vel);

        double currentVelocity = launcherTop.getVelocity();

        double power = (TuneShooterVelocity.kA * TuneShooterVelocity.vel) + shooterController.calculate(currentVelocity);

        power = Math.max(-1, Math.min(power, 1));

        launcherBottom.setPower(power);
        launcherTop.setPower(power);

        if (gamepad1.y) {
            hammerShooter.setPosition(0);
        } else {
            hammerShooter.setPosition(0.6);
        }

        hood.setPosition(servoPos);

        dashboardTelemetry.addData("Hood angle", hood.getPosition());
        dashboardTelemetry.addData("velocity", launcherBottom.getVelocity());
        dashboardTelemetry.addData("power", power);
        dashboardTelemetry.update();
    }
}