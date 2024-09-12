package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

public class OmniPID extends LinearOpMode {

    // Declare motor and IMU objects
    private DcMotor frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
    private IMU imu;
    
    // PID control variables
    private double kP = 0.01, kI = 0.0, kD = 0.0; // Adjust these values based on tuning
    private double integral = 0, lastError = 0;
    private ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() {
        // Initialize motors and IMU
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");

        imu = hardwareMap.get(IMU.class, "imu");

        // Set motors' direction so that they spin correctly
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        // Reset IMU
        imu.resetYaw();

        // Wait for the start button
        waitForStart();

        while (opModeIsActive()) {
            // Get joystick inputs for translation and rotation
            double forward = -gamepad1.left_stick_y;  // Forward/Backward
            double strafe = gamepad1.left_stick_x;    // Left/Right
            double rotate = gamepad1.right_stick_x;   // Rotation

            // Get current heading (yaw) from IMU
            double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            // Target heading is based on user input (rotate value)
            double targetHeading = rotate * 30; // scale rotation by factor (optional)

            // Compute PID correction for heading control
            double correction = pidController(targetHeading, currentHeading);

            // Calculate motor powers for omni-drive movement
            double flPower = forward + strafe + correction;
            double frPower = forward - strafe - correction;
            double blPower = forward - strafe + correction;
            double brPower = forward + strafe - correction;

            // Normalize motor powers if any exceeds the max power
            double maxPower = Math.max(Math.abs(flPower), Math.abs(frPower));
            maxPower = Math.max(maxPower, Math.abs(blPower));
            maxPower = Math.max(maxPower, Math.abs(brPower));

            if (maxPower > 1.0) {
                flPower /= maxPower;
                frPower /= maxPower;
                blPower /= maxPower;
                brPower /= maxPower;
            }

            // Set the motor powers
            frontLeftMotor.setPower(flPower);
            frontRightMotor.setPower(frPower);
            backLeftMotor.setPower(blPower);
            backRightMotor.setPower(brPower);

            // Display telemetry
            telemetry.addData("Heading", currentHeading);
            telemetry.addData("Target Heading", targetHeading);
            telemetry.addData("Correction", correction);
            telemetry.update();
        }
    }

    // PID controller for heading correction
    private double pidController(double targetHeading, double currentHeading) {
        double error = targetHeading - currentHeading;
        integral += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;
        timer.reset();
        return kP * error + kI * integral + kD * derivative;
    }
}
