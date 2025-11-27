package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@TeleOp(name="VoltageBasedSpeedTest", group="test")
public class VoltageBasedSpeedTest extends LinearOpMode {

    DcMotorEx motor;

    @Override
    public void runOpMode() throws InterruptedException {

        motor = hardwareMap.get(DcMotorEx.class, "motor");

        waitForStart();

        double desiredVoltage = 10.0;
        double batteryVoltage = getBatteryVoltage();
        double power = desiredVoltage / batteryVoltage;

        motor.setPower(power);

        sleep(2000); // esperar a que la velocidad se estabilice

        double measuredVelocity = motor.getVelocity();
        telemetry.addData("Measured Velocity", measuredVelocity);
        telemetry.addData("Battery Voltage", batteryVoltage);
        telemetry.addData("Applied Power", power);
        telemetry.update();

        while (opModeIsActive()) {}
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

