package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystems.Shooter.ShooterIO;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.VisionIO;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name="TuneHood", group="test")
public class TuneHood extends OpMode {

    private Servo hood;
    public static double servoPos = 0;

    private boolean logged = false;
    public static double distance;

    private VisionIO vision;

    private DcMotorEx launcherTop;
    private DcMotorEx launcherBottom;


    @Override
    public void init() {
        hood = hardwareMap.get(Servo.class, "shooterPitch");
        vision = new VisionIO(hardwareMap, new ShooterIO(hardwareMap), telemetry);

        launcherTop = hardwareMap.get(DcMotorEx.class, "launcherTop");
        launcherBottom = hardwareMap.get(DcMotorEx.class, "launcherBottom");

        launcherBottom.setDirection(DcMotorSimple.Direction.REVERSE);

        launcherTop.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(TuneShooterVelocity.kP, TuneShooterVelocity.kI, 0, TuneShooterVelocity.kF));
        launcherBottom.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(TuneShooterVelocity.kP, TuneShooterVelocity.kI, 0, TuneShooterVelocity.kF));

        launcherTop.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherBottom.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        launcherTop.setVelocity(TuneShooterVelocity.vel);
        launcherBottom.setVelocity(TuneShooterVelocity.vel);

        AprilTagDetection tagDetection = vision.getTagBySpecificId(20);
        hood.setPosition(servoPos);

        if (tagDetection != null){
            distance = Math.hypot(tagDetection.ftcPose.x, tagDetection.ftcPose.y);
        }

        if (gamepad1.a && !logged) {
            telemetry.addData("HoodData", servoPos + "," + distance);
            logged = true;
        }
        if (!gamepad1.a) logged = false;

        telemetry.addData("Hood angle", hood.getPosition());
        vision.displayDetectionTelemetry(tagDetection);
        telemetry.update();
    }
}
