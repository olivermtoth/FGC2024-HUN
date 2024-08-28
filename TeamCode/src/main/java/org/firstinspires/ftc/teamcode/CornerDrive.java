package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


@TeleOp(name="Corner Drive", group="Examples")
public class CornerDrive extends LinearOpMode {

    DcMotor front_left_wheel;
    DcMotor back_left_wheel;
    DcMotor back_right_wheel;
    DcMotor front_right_wheel;
    IMU imu;
    double Kp = 0.025;  // Proportional gain
    double Ki = 0.0;   // Integral gain
    double Kd = 0.00;  // Derivative gain
    ElapsedTime PIDTimer = new ElapsedTime();


    @Override
    public void runOpMode() {

        front_left_wheel = hardwareMap.dcMotor.get("fl");
        back_left_wheel = hardwareMap.dcMotor.get("bl");
        back_right_wheel = hardwareMap.dcMotor.get("br");
        front_right_wheel = hardwareMap.dcMotor.get("fr");

        front_left_wheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_left_wheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_right_wheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        front_right_wheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        imu = hardwareMap.get(IMU.class, "imu");

        front_left_wheel.setDirection(DcMotor.Direction.REVERSE);
        back_left_wheel.setDirection(DcMotor.Direction.FORWARD);
        front_right_wheel.setDirection(DcMotor.Direction.FORWARD);
        back_right_wheel.setDirection(DcMotor.Direction.FORWARD);

        front_left_wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left_wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right_wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right_wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        IMU.Parameters myIMUparameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.LEFT
                )
        );

        imu.initialize(myIMUparameters);
        imu.resetYaw();

        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();

        double targetHeading = orientation.getYaw(AngleUnit.DEGREES);// Desired heading
        telemetry.addData("Target heading", targetHeading);
        telemetry.addLine("OK 2 RUN");
        telemetry.update();

        double previousError = 0.0;
        telemetry.update();

        waitForStart();



        while (opModeIsActive()) {
            // if(PIDTimer.seconds() == 0.0){
            //     PIDTimer.startTime();
            // }
            // orientation = imu.getRobotYawPitchRollAngles();
            // double currentHeading = orientation.getYaw(AngleUnit.DEGREES);
            // double error = angleWrap(targetHeading - currentHeading);
            // double derivative = (error - previousError)/ PIDTimer.seconds();
            // double correction = Kp * error + Kd * derivative;
            // PIDTimer.reset();
            // front_left_wheel.setPower(correction);
            // back_left_wheel.setPower(correction);
            // back_right_wheel.setPower(correction);
            // front_right_wheel.setPower(correction);
            // PIDTimer.startTime();

            double drive = -gamepad1.left_stick_y;  // Forward/backward movement
            double strafe = gamepad1.left_stick_x;  // Left/right movement
            double turn = 0;   // Rotation

            double frontLeftPower = drive + strafe + turn;
            double frontRightPower = drive - strafe - turn;
            double backLeftPower = drive - strafe + turn;
            double backRightPower = drive + strafe - turn;

            double max = Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                    Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)));

            if (max > 1.0) {
                frontLeftPower /= max;
                frontRightPower /= max;
                backLeftPower /= max;
                backRightPower /= max;
            }

            front_left_wheel.setPower(frontLeftPower);
            front_right_wheel.setPower(frontRightPower);
            back_left_wheel.setPower(backLeftPower);
            back_right_wheel.setPower(backRightPower);






            // double power = .5;
            // if(gamepad1.dpad_up){ //Forward
            //     front_left_wheel.setPower(-power);
            //     back_left_wheel.setPower(-power);
            //     back_right_wheel.setPower(-power);
            //     front_right_wheel.setPower(-power);
            // }
            // else if(gamepad1.dpad_left){ //Left
            //     front_left_wheel.setPower(power);
            //     back_left_wheel.setPower(-power);
            //     back_right_wheel.setPower(power);
            //     front_right_wheel.setPower(-power);
            // }
            // else if(gamepad1.dpad_down){ //Back
            //     front_left_wheel.setPower(power);
            //     back_left_wheel.setPower(power);
            //     back_right_wheel.setPower(power);
            //     front_right_wheel.setPower(power);
            // }
            // else if(gamepad1.dpad_right){ //Right
            //     front_left_wheel.setPower(-power);
            //     back_left_wheel.setPower(power);
            //     back_right_wheel.setPower(-power);
            //     front_right_wheel.setPower(power);
            // }
            // else if(Math.abs(gamepad1.right_stick_x) > 0){ //Rotation
            //     front_left_wheel.setPower(-gamepad1.right_stick_x);
            //     back_left_wheel.setPower(-gamepad1.right_stick_x);
            //     back_right_wheel.setPower(gamepad1.right_stick_x);
            //     front_right_wheel.setPower(gamepad1.right_stick_x);
            // }
            // else{
            //     // front_left_wheel.setPower(0);
            //     // back_left_wheel.setPower(0);
            //     // back_right_wheel.setPower(0);
            //     // front_right_wheel.setPower(0);
            // }
        }
    }

    public double angleWrap(double degrees){
        while(degrees > 180){
            degrees -= 360;
        }
        while(degrees < -180){
            degrees += 360;
        }
        return degrees;
    }
}