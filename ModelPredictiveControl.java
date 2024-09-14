package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ModelPredictiveControl extends LinearOpMode {

    // Declare motor and IMU objects
    private DcMotor frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
    private IMU imu;

    // Robot state [x, y, heading] and control inputs [vx, vy, w]
    private double[] state = {0, 0, 0}; // [x, y, heading]
    private double[] controlInputs = {0, 0, 0}; // [vx, vy, angular velocity]
    
    // MPC parameters
    private double predictionHorizon = 10; // Number of steps to predict
    private double dt = 0.1; // Time step for predictions
    private double controlLimit = 1.0; // Max control input
    
    // Cost weights
    private double positionWeight = 1.0;
    private double headingWeight = 1.0;
    private double controlWeight = 0.1;

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
            state[2] = currentHeading; // Update heading in state vector

            // Set target positions based on user input
            double targetX = state[0] + forward;
            double targetY = state[1] + strafe;
            double targetHeading = currentHeading + rotate * 30; // Rotate scaled

            // Perform MPC optimization to get optimal control inputs
            controlInputs = mpcOptimization(targetX, targetY, targetHeading, state);

            // Apply the control inputs to the motors (vx, vy, angular velocity)
            applyMotorPower(controlInputs[0], controlInputs[1], controlInputs[2]);

            // Display telemetry
            telemetry.addData("Heading", currentHeading);
            telemetry.addData("Target X", targetX);
            telemetry.addData("Target Y", targetY);
            telemetry.addData("Control Input [vx, vy, w]", controlInputs);
            telemetry.update();
        }
    }

    // MPC optimization routine
    private double[] mpcOptimization(double targetX, double targetY, double targetHeading, double[] currentState) {
        // Initialize control inputs
        double[] optimalControl = {0, 0, 0}; // [vx, vy, w]

        double minCost = Double.MAX_VALUE;

        // Search over control inputs to minimize cost function
        for (double vx = -controlLimit; vx <= controlLimit; vx += 0.1) {
            for (double vy = -controlLimit; vy <= controlLimit; vy += 0.1) {
                for (double w = -controlLimit; w <= controlLimit; w += 0.1) {

                    // Predict future state based on control inputs
                    double[] predictedState = predictState(currentState, vx, vy, w);

                    // Calculate cost (position error + heading error + control effort)
                    double cost = calculateCost(predictedState, targetX, targetY, targetHeading, vx, vy, w);

                    // Update optimal control if cost is lower
                    if (cost < minCost) {
                        minCost = cost;
                        optimalControl[0] = vx;
                        optimalControl[1] = vy;
                        optimalControl[2] = w;
                    }
                }
            }
        }

        return optimalControl;
    }

    // Predict the robot's future state given current state and control inputs
    private double[] predictState(double[] currentState, double vx, double vy, double w) {
        double[] nextState = new double[3];

        // Predict the next state based on current state and control inputs
        nextState[0] = currentState[0] + vx * dt; // Update x position
        nextState[1] = currentState[1] + vy * dt; // Update y position
        nextState[2] = currentState[2] + w * dt;  // Update heading

        return nextState;
    }

    // Calculate the cost of a predicted state and control effort
    private double calculateCost(double[] predictedState, double targetX, double targetY, double targetHeading, double vx, double vy, double w) {
        // Position error
        double positionError = positionWeight * (Math.pow(predictedState[0] - targetX, 2) + Math.pow(predictedState[1] - targetY, 2));

        // Heading error
        double headingError = headingWeight * Math.pow(predictedState[2] - targetHeading, 2);

        // Control effort
        double controlEffort = controlWeight * (Math.pow(vx, 2) + Math.pow(vy, 2) + Math.pow(w, 2));

        // Total cost
        return positionError + headingError + controlEffort;
    }

    // Apply the control inputs to the motors
    private void applyMotorPower(double vx, double vy, double w) {
        // Calculate motor powers for omni-directional movement
        double flPower = vx + vy + w;
        double frPower = vx - vy - w;
        double blPower = vx - vy + w;
        double brPower = vx + vy - w;

        // Set motor powers
        frontLeftMotor.setPower(flPower);
        frontRightMotor.setPower(frPower);
        backLeftMotor.setPower(blPower);
        backRightMotor.setPower(brPower);
    }
}
