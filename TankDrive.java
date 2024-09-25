package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

import global.first.FeedingTheFutureGameDatabase;


@TeleOp(name="Tank Drive Test")
public class TankDrive extends LinearOpMode {
    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private DcMotor linear1, linear2;
    private Servo leftWall, rightWall, leftGate, rightGate, backGate;
    private DistanceSensor distance;
    private final int[] l1Height = {2100, 5200, 5200, 5200};
    private final int[] l2Height = {0, 1500, 4400, 8600};
    private AprilTagLibrary aprilTagLibrary;
    private AprilTagPoseFtc pose;
    private double limits[] = {1,1, 0.8, 0.5};
    private List<AprilTagDetection> currentDetections;

    /**
     * {@link #aprilTag} is the variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    /**
     * {@link #visionPortal} is the variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {

        initAprilTag();

        frontLeft = hardwareMap.get(DcMotorEx.class, "fl"); //1
        backLeft = hardwareMap.get(DcMotorEx.class, "bl"); //3
        frontRight = hardwareMap.get(DcMotorEx.class, "fr"); //0
        backRight = hardwareMap.get(DcMotorEx.class, "br"); //2

        linear1 = hardwareMap.get(DcMotor.class, "linear1");
        linear2 = hardwareMap.get(DcMotor.class, "linear2");

        leftWall = hardwareMap.get(Servo.class, "lw");
        rightWall = hardwareMap.get(Servo.class, "rw");

        leftGate = hardwareMap.get(Servo.class, "lg");
        rightGate = hardwareMap.get(Servo.class, "rg");

        backGate = hardwareMap.get(Servo.class, "bg");

        distance = hardwareMap.get(DistanceSensor.class, "distance");

        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        linear1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linear2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // linear1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // linear2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        // rightWall.setPosition(1);
        // leftWall.setPosition(1);

        // rightGate.setPosition(0);
        // leftGate.setPosition(1);
        // backGate.setPosition(1);


        double leftY;
        double leftX;
        double rightX;
        boolean lgTemp = false;
        boolean rgTemp = false;
        boolean xTemp = false;
        boolean circlePressed = false;
        int heightIndex = 0;
        boolean firstTimeRun = true;
        double distanceToGo;
        boolean manualLinear = false;


        waitForStart();
        while (opModeIsActive()) {

            if(firstTimeRun){
                linear1.setTargetPosition(l1Height[heightIndex]);
                linear1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                linear1.setPower(1);

                while((linear1.isBusy()) && !isStopRequested()) {
                    // Let the drive team see that we're waiting on the motor
                    telemetry.addData("Status", "Waiting for the motor to reach its target");
                    telemetry.update();
                }

                rightGate.setPosition(1);
                leftGate.setPosition(0);
                sleep(1500);
                rightWall.setPosition(1);
                leftWall.setPosition(0);
                firstTimeRun = false;
            }

            telemetryAprilTag();



            leftY = gamepad1.left_stick_y;
//            leftX = gamepad1.left_stick_x;
            rightX = gamepad1.right_stick_x*0.6;

            // double leftPower = leftY+leftX;
            // double rightPower = leftY-leftX;

            if(rightX==0){
                backLeft.setPower(leftY*limits[heightIndex]);
                backRight.setPower(leftY*limits[heightIndex]);
                frontRight.setPower(leftY*limits[heightIndex]);
                frontLeft.setPower(leftY*limits[heightIndex]);
            }
            else{
                backLeft.setPower(rightX*limits[heightIndex]);
                backRight.setPower(-rightX*limits[heightIndex]);
                frontRight.setPower(-rightX*limits[heightIndex]);
                frontLeft.setPower(rightX*limits[heightIndex]);
            }

            if(manualLinear){
                 linear1.setPower(gamepad2.left_stick_y);
                 linear2.setPower(gamepad2.right_stick_y);
            }
            else{
                if(gamepad2.circle & !circlePressed){
                    heightIndex = (heightIndex + 1)%4;
                    circlePressed = true;
                }
                else if(!gamepad2.circle & circlePressed){
                    circlePressed = false;
                }

                if(gamepad2.triangle){
                    linear1.setTargetPosition(l1Height[heightIndex]);
                    linear1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    linear1.setPower(1);

                    linear2.setTargetPosition(l2Height[heightIndex]);
                    linear2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    linear2.setPower(1);
                }

                if(gamepad2.square){
                    heightIndex = 0;
                    linear1.setTargetPosition(l1Height[heightIndex]);
                    linear1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    linear1.setPower(1);

                    linear2.setTargetPosition(l2Height[heightIndex]);
                    linear2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    linear2.setPower(1);
                }
            }


            // if(gamepad2.dpad_up){
            //     rightWall.setPosition(1);
            //     leftWall.setPosition(0);

            // }

            if(gamepad2.dpad_left){
                backGate.setPosition(0);
            }

            if(gamepad2.dpad_right){
                backGate.setPosition(1);
            }

            // if(gamepad2.dpad_down){
            //     rightWall.setPosition(0);
            //     leftWall.setPosition(1);
            // }

//            telemetry.addData("linear1", linear1.getCurrentPosition());
//            telemetry.addData("linear2", linear2.getCurrentPosition());
            telemetry.addData("Position", heightIndex);

            if(gamepad2.left_bumper && !lgTemp){
                lgTemp = true;
                if(leftGate.getPosition() == 0){
                    leftGate.setPosition(0.65);
                }
                else{
                    leftGate.setPosition(0);
                }
            }
            else if(!gamepad2.left_bumper && lgTemp){
                lgTemp = false;
            }


            if(gamepad2.right_bumper && !rgTemp){
                rgTemp = true;
                if(rightGate.getPosition() == 0.75){
                    rightGate.setPosition(1);
                }
                else{
                    rightGate.setPosition(0.75);
                }
            }
            else if(!gamepad2.right_bumper && rgTemp){
                rgTemp = false;
            }

             if(gamepad2.ps & !xTemp){
                 if(linear1.getMode() == DcMotor.RunMode.RUN_TO_POSITION && linear2.getMode() == DcMotor.RunMode.RUN_TO_POSITION){
                     linear1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                     linear2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                     manualLinear = true;
                 }
             }
             else if(!gamepad2.ps & xTemp){
                 xTemp = false;
             }

            if(gamepad2.x && !xTemp){
                xTemp = true;
                if(backGate.getPosition() == 0){
                    rightGate.setPosition(1);
                }
                else{
                    rightGate.setPosition(0);
                }
            }
            else if(!gamepad2.x && xTemp){
                xTemp = false;
            }

            distanceToGo = distance.getDistance(DistanceUnit.CM);
            if(distanceToGo < 12){
                gamepad1.rumbleBlips(20);
                gamepad2.rumbleBlips(20);
            }
            else if (distanceToGo < 20 ){
                gamepad1.rumble(600);
                gamepad2.rumble(600);
            }
            else{
                gamepad1.stopRumble();
                gamepad2.stopRumble();
            }

            //telemetry.addData("rg" ,rightGate.getPosition());
            //telemetry.addData("lg", leftGate.getPosition());
            //telemetry.addData("lw", leftWall.getPosition());
            //telemetry.addData("rw", rightWall.getPosition());
            //telemetry.addData("bg", backGate.getPosition());
            telemetry.addData("Distance", distance.getDistance(DistanceUnit.CM));
            telemetry.addData("Manual Linear",manualLinear);

            telemetry.update();
        }

    }

    private void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()
                .setTagLibrary(FeedingTheFutureGameDatabase.getFeedingTheFutureTagLibrary())
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)

                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the webcam (name is assumed to be "Webcam 1")
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        visionPortal.setProcessorEnabled(aprilTag, true);

    }   // end method initAprilTag()


    /**
     * Function to add telemetry about AprilTag detections.
     */
    private void telemetryAprilTag() {

        currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
//                telemetry.addLine(String.format("\n==== (ID %d)", detection.id));
                telemetry.addLine(detection.metadata.name);
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (cm)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                if(Math.abs(detection.ftcPose.x) < 70){
                    gamepad1.rumble(500);
                }
//                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
//                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (cm, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
//        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
//        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
//        telemetry.addLine("RBE = Range, Bearing & Elevation");

    }   // end method telemetryAprilTag()
}
