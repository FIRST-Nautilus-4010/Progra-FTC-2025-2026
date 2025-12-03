package org.firstinspires.ftc.teamcode.Test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Subsystems.Shooter.ShooterIO;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.VisionIO;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name="TuneHood", group="test")
@Config
public class TuneHood extends OpMode {

    private Servo hood;
    public static double servoPos = 0;

    private boolean logged = false;
    public static double distance;

    private VisionIO vision;

    private DcMotorEx launcherTop;
    private DcMotorEx launcherBottom;
    private boolean alreadyPressedY = false;

    private Servo hammerShooter;
    public static double desiredVoltage;

    double power = 0;

    @Override
    public void init() {
        hood = hardwareMap.get(Servo.class, "shooterPitch");
        vision = new VisionIO(hardwareMap, new ShooterIO(hardwareMap), telemetry);

        launcherTop = hardwareMap.get(DcMotorEx.class, "launcherTop");
        launcherBottom = hardwareMap.get(DcMotorEx.class, "launcherBottom");

        launcherBottom.setDirection(DcMotorSimple.Direction.REVERSE);

        launcherTop.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launcherBottom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        hammerShooter = hardwareMap.get(Servo.class, "hammerS");
        double batteryVoltage = getBatteryVoltage();
        power = desiredVoltage / batteryVoltage;
    }

    @Override
    public void loop() {
        launcherTop.setPower(-power);
        launcherBottom.setPower(-power);

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

        if (gamepad1.y && !alreadyPressedY){
            hammerShooter.setPosition(0);
        }else{
            hammerShooter.setPosition(1);
        }

        telemetry.addData("Hood angle", hood.getPosition());
        telemetry.addData("velocity", launcherBottom.getVelocity());
        telemetry.addData("power", power);
        vision.displayDetectionTelemetry(tagDetection);
        telemetry.update();
    }

    private double getBatteryVoltage() {
        double result = 0;
        for (VoltageSensor sensor : hardwareMap.getAll(VoltageSensor.class)) {
            double voltage = sensor.getVoltage();
            if (voltage > result) {
                result = voltage;
            }
        }
        return result;
    }
}