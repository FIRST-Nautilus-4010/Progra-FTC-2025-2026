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

    private final HardwareMap hardwareMap;

    public ShooterIO(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

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
            double motorTicks = 28.0;        // ticks por vuelta del motor
            double ratio = 36.0 * 1.9;       // relación total (68.4)
            double ticksPerRevFinal = motorTicks * ratio; // 1915.2

            int target = (int) Math.round((angle / (2 * Math.PI)) * ticksPerRevFinal);

            yawMotor.setTargetPosition(target);
            yawMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            yawMotor.setPower(1);
            return;
        }

        stopYaw();
    }

    public HardwareMap getHardwareMap() {
        return hardwareMap;
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
        double tps = 28 * (vel / 60);

        launcherMotorTop.setVelocity(-tps);
        launcherMotorBottom.setVelocity(tps);
    }

    public double getYaw() {
        return ((double) yawMotor.getCurrentPosition() / (28 * (135 / 70) * 36)) * (2 * Math.PI);
    }

    public double getPitch() {
        return (((1 - pitchServo.getPosition())) * Math.toRadians(65)) + Math.toRadians(10);
        //return pitchServo.getPosition();
    }

    public void setYawPower(double power) {;
        yawMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        yawMotor.setPower(power);
    }

    public double getVel() {
        return 60 * (-launcherMotorTop.getVelocity() / 28);
    }
}
