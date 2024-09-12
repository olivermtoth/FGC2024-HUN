package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "OnmiDrive PID")
public class OmniPID extends LinearOpMode {

    // Declare motor and IMU objects
    private DcMotor frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
    private IMU imu;
    
    // PID control variables
    private double kP = 0.2, kI = 0.0, kD = 0.1; // Adjust these values based on tuning
    private double integral = 0, lastError = 0, targetHeading;
    private ElapsedTime timer = new ElapsedTime();
    private final double PI = 3.141592653589793;

    @Override
    public void runOpMode() {
        // Initialize motors and IMU
        frontLeftMotor = hardwareMap.get(DcMotor.class, "drive1");
        frontRightMotor = hardwareMap.get(DcMotor.class, "drive0");
        backLeftMotor = hardwareMap.get(DcMotor.class, "drive3");
        backRightMotor = hardwareMap.get(DcMotor.class, "drive2");

        imu = hardwareMap.get(IMU.class, "imu");

        // Set motors' direction so that they spin correctly
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);
        
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        
        // Reset IMU
        imu.resetYaw();
        targetHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Wait for the start button
        waitForStart();

        while (opModeIsActive()) {
            // Get joystick inputs for translation and rotation
            double forward = -gamepad1.left_stick_y;  // Forward/Backward
            double strafe = gamepad1.left_stick_x;    // Left/Right
            double rotate = gamepad1.right_stick_x;   // Rotation

            // Get current heading (yaw) from IMU
            double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);


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
            telemetry.addData("Time", timer.seconds());
            telemetry.addData("Heading", currentHeading);
            telemetry.addData("Target Heading", targetHeading);
            telemetry.addData("Correction", correction);
            telemetry.update();
        }
    }

    // PID controller for heading correction
    private double pidController(double targetHeading, double currentHeading) {
        double error = angleWrap(targetHeading - currentHeading);
        integral += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;
        timer.reset();
        return kP * error + kI * integral + kD * derivative;
    }
    
     public double angleWrap(double degrees){
        while(degrees > PI){
            degrees -= 2*PI;
        }
        while(degrees < -PI){
            degrees += 2*PI;
        }
        return degrees;
    }
}
