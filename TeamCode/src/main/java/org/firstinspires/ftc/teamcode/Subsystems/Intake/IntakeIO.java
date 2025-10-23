package org.firstinspires.ftc.teamcode.Subsystems.Intake;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeIO {
    private final DigitalChannel stage1LimitSwitch;
    private final DigitalChannel stage2LimitSwitch;
    private final DigitalChannel stage3LimitSwitch;

    private final DcMotorEx motor;



    public IntakeIO (HardwareMap hardwareMap) {
        stage1LimitSwitch = hardwareMap.get(DigitalChannel.class, "stage1");
        stage2LimitSwitch = hardwareMap.get(DigitalChannel.class, "stage2");
        stage3LimitSwitch = hardwareMap.get(DigitalChannel.class, "stage3");

        motor = hardwareMap.get(DcMotorEx.class, "intake");
    }

    public void setVel(double vel) {
        motor.setVelocity(vel);
    }

    public boolean onStage1() {
        return stage1LimitSwitch.getState();
    }

    public boolean onStage2() {
        return stage2LimitSwitch.getState();
    }

    public boolean onStage3() {
        return stage3LimitSwitch.getState();
    }

    public double getVel() {
        return motor.getVelocity();
    }

}
