package org.firstinspires.ftc.teamcode.Subsystems.Shooter;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ShooterIO {
    private final DcMotorEx yawMotor;
    private final Servo pitchServo;

    private final DcMotorEx launcherMotorTop;
    private final DcMotorEx launcherMotorBottom;

    public ShooterIO(HardwareMap hardwareMap) {
        yawMotor = hardwareMap.get(DcMotorEx.class, "shooterYaw");
        pitchServo = hardwareMap.get(Servo.class, "shooterPitch");

        launcherMotorTop = hardwareMap.get(DcMotorEx.class, "launcherTop");
        launcherMotorBottom = hardwareMap.get(DcMotorEx.class, "launcherBottom");
        launcherMotorTop.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherMotorBottom.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        yawMotor.setPositionPIDFCoefficients(200);
        yawMotor.setTargetPositionTolerance(25);
    }

    public void setYaw(double angle) {
        yawMotor.setTargetPosition((int) Math.round((angle / (2 * Math.PI)) * 576));
        yawMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        yawMotor.setPower(1);
    }

    public void setPitch(double angle) {
        pitchServo.setPosition(angle);
    }

    public void setVel(double vel) {
        launcherMotorTop.setVelocity(-vel);
        launcherMotorBottom.setVelocity(vel);
    }

    public double getYaw() {
        return ((double) yawMotor.getCurrentPosition() / 576) * (2 * Math.PI);
    }

    public double getPitch() {
        return pitchServo.getPosition() * 1;
    }

    public double getVel() {
        return -launcherMotorTop.getVelocity();
    }
}
