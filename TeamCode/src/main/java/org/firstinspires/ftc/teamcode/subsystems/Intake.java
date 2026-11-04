package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {
    private final DcMotorEx intakeMotor;
    private final Servo leftBar, rightBar;

    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        leftBar = hardwareMap.get(Servo.class, "leftBar");
        rightBar = hardwareMap.get(Servo.class, "rightBar");

        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        leftBar.setDirection(Servo.Direction.REVERSE);
        rightBar.setDirection(Servo.Direction.FORWARD);
        leftBar.setPosition(0.1);
        rightBar.setPosition(0.1);
    }

    public void start() {
        intakeMotor.setPower(1);
    }

    public void stop() {
        intakeMotor.setPower(0);
    }

    public void reverse() {
        intakeMotor.setPower(-0.75);
    }

    public void flowerIntake() {
        leftBar.setPosition(0.9);
        rightBar.setPosition(0.9);
        intakeMotor.setPower(1);
    }

    public void barUp() {
        leftBar.setPosition(0.1);
        rightBar.setPosition(0.1);
    }
}
