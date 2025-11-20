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

        yawMotor.setPositionPIDFCoefficients(100);
        yawMotor.setTargetPositionTolerance(10);
    }

    public void setYaw(double angle) {

        if (!(Math.abs(angle) > Math.PI/2)) {
            yawMotor.setTargetPosition((int) Math.round((angle / (2 * Math.PI)) * (28 * 1.98 * 36)));
            yawMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            yawMotor.setPower(1);
            return;
        }

        stopYaw();
    }

    public double getYawVel() {
        return yawMotor.getVelocity();
    }

    public void stopYaw() {
        yawMotor.setVelocity(0);
    }

    public void setPitch(double angle) {
        if (angle <= Math.toRadians(10)) {
            pitchServo.setPosition(1);
            return;
        }

        if (angle >= Math.toRadians(75)) {
            pitchServo.setPosition(0);
            return;
        }

        pitchServo.setPosition(1 - (((angle - Math.toRadians(10)) / Math.toRadians(65))));

        //pitchServo.setPosition(angle);
    }

    public void setVel(double vel) {
        launcherMotorTop.setVelocity(-vel);
        launcherMotorBottom.setVelocity(vel);
    }

    public double getYaw() {
        return ((double) yawMotor.getCurrentPosition() / (28 * 1.98 * 36)) * (2 * Math.PI);
    }

    public double getPitch() {
        return (((1 - pitchServo.getPosition())) * Math.toRadians(65)) + Math.toRadians(10);
        //return pitchServo.getPosition();
    }

    public double getVel() {
        return -launcherMotorTop.getVelocity();
    }
}
