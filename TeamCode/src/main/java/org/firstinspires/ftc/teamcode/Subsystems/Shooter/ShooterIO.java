package org.firstinspires.ftc.teamcode.Subsystems.Shooter;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ShooterIO {
    private final DcMotorEx yawMotor;
    private final DcMotorEx pitchMotor;

    private final DcMotorEx launcherMotorTop;
    private final DcMotorEx launcherMotorBottom;

    public ShooterIO(HardwareMap hardwareMap) {
        yawMotor = hardwareMap.get(DcMotorEx.class, "shooterYaw");
        pitchMotor = hardwareMap.get(DcMotorEx.class, "pitchYaw");

        launcherMotorTop = hardwareMap.get(DcMotorEx.class, "launcherTop");
        launcherMotorBottom = hardwareMap.get(DcMotorEx.class, "launcherBottom");
    }

    public void setYaw(double angle) {
        yawMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        yawMotor.setTargetPosition((int) angle / 1);
        yawMotor.setPower(1);
    }

    public void setPitch(double angle) {
        pitchMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pitchMotor.setTargetPosition((int) (angle / (2 * Math.PI)));
        pitchMotor.setPower(1);
    }

    public void setVel(double vel) {
        launcherMotorTop.setVelocity(vel);
        launcherMotorBottom.setVelocity(-vel);
    }

    public double getYaw() {
        return yawMotor.getCurrentPosition() * 1;
    }

    public double getPitch() {
        return pitchMotor.getCurrentPosition() * 1;
    }

    public double getVel() {
        return launcherMotorTop.getVelocity();
    }
}
