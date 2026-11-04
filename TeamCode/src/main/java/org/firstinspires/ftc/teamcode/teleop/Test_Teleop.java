package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.PID;

@TeleOp(name = "Drive", group = "Test")
public class Test_Teleop extends OpMode {
    private MecanumDrive drive;
    private double commandDegrees = 0.0;
    double rx;
    PID turnController = new PID(0.02, 0, 0.0);

    public void init() {
        drive = new MecanumDrive(hardwareMap, new Pose2d(0,0, 0));
    }

    public void loop() {
        double y = gamepad1.right_stick_y;
        double x = gamepad1.right_stick_x;

        if (gamepad1.left_stick_x > 0.15) {
            commandDegrees -= 0.5;
        } else if (gamepad1.left_stick_x < -0.15) {
            commandDegrees += 0.5;
        }

        // Auto turn with d-pad
        if(gamepad1.dpad_down) {
            if (Math.toDegrees(drive.localizer.getPose().heading.toDouble()) < 0) {
                commandDegrees = -180;
            } else {
                commandDegrees = 180;
            }
        } else if (gamepad1.dpad_up) {
            commandDegrees = 0;
        } else if (gamepad1.dpad_left) {
            commandDegrees = 90;
        } else if (gamepad1.dpad_right) {
            commandDegrees = -90;
        }

        drive.localizer.update();

        rx = turnController.calculate(Math.toDegrees(drive.localizer.getPose().heading.toDouble()), commandDegrees);

        // ********************* Driving Math ***** Do NOT Change *********************
        double botHeading = drive.localizer.getPose().heading.toDouble();

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(botHeading);
        double rotY = x * Math.sin(botHeading) + y * Math.cos(-botHeading);

        rotY = rotY * 1.05;  // Counteract imperfect strafing

        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(rotX, rotY), rx));
    }
}
