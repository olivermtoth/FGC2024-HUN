

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp

public class AutoFTCTest extends LinearOpMode {
    private DcMotor FL;
    private DcMotor FR;
    private DcMotor BL;
    private DcMotor BR;
    private DcMotor linear;
    private DcMotor arm;

    /*
    public void stopMvmt() {
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL.setTargetPosition(FL.getCurrentPosition());
        FR.setTargetPosition(FR.getCurrentPosition());
        BL.setTargetPosition(BL.getCurrentPosition());
        BR.setTargetPosition(BR.getCurrentPosition());
        FL.setPower(1);
        FR.setPower(1);
        BL.setPower(1);
        BR.setPower(1);
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void stopLeft() {
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL.setTargetPosition(FL.getCurrentPosition());
        BL.setTargetPosition(BL.getCurrentPosition());
        FL.setPower(1);
        BL.setPower(1);
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void stopRight() {
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setTargetPosition(FR.getCurrentPosition());
        BR.setTargetPosition(BR.getCurrentPosition());
        FR.setPower(1);
        BR.setPower(1);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void moveForward(double power) {
        FL.setPower(power);
        FR.setPower(power);
        BL.setPower(power);
        BR.setPower(power);
    }
    public void tankDrive(double left, double right) {
        FL.setPower(left);
        FR.setPower(right);
        BL.setPower(left);
        BR.setPower(right);
    }
    */
    public void runOpMode() {
        // actuators
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        linear = hardwareMap.get(DcMotor.class, "linear");
        arm = hardwareMap.get(DcMotor.class, "arm");
        FL.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotor.Direction.REVERSE);

        // camera vision
        AprilTagProcessor myAprilTagProcessor;
        AprilTagLibrary myAprilTagLibrary;
        myAprilTagLibrary = AprilTagGameDatabase.getCurrentGameTagLibrary();
        myAprilTagProcessor = new AprilTagProcessor.Builder()
            .setTagLibrary(myAprilTagLibrary)
            .setDrawTagID(true)
            .setDrawTagOutline(true)
            .setDrawAxes(true)
            .setDrawCubeProjection(true)
            .build();
        
        VisionPortal myVisionPortal;
        myVisionPortal = new VisionPortal.Builder()
            .setCamera(hardwareMap.get(WebcamName.class, "logi720p"))
            .addProcessor(myAprilTagProcessor)
//            .setCameraResolution(new android.util.Size(864, 480))   // 16:9
            .setCameraResolution(new android.util.Size(640, 480))   //  4:3
            .setStreamFormat(VisionPortal.StreamFormat.YUY2)
            .enableLiveView(true)
            .setAutoStopLiveView(true)
            .build();

        // variables
        double leftY;
        double leftX;
        double rightY;
        double rightX;
        double RTrigger;
        double LTrigger;

        // automated
        
        
        waitForStart();
        while (opModeIsActive()) {
            leftY = Math.pow(gamepad1.left_stick_y, 2) * Math.signum(gamepad1.left_stick_y);
            leftX = Math.pow(gamepad1.left_stick_x, 2) * Math.signum(gamepad1.left_stick_x);
            rightY = Math.pow(gamepad1.right_stick_y, 2) * Math.signum(gamepad1.right_stick_y);
            rightX = Math.pow(gamepad1.right_stick_x, 2) * Math.signum(gamepad1.right_stick_x);
            RTrigger = Math.pow(gamepad1.right_trigger, 2);
            LTrigger = Math.pow(gamepad1.left_trigger, 2);
            
//            if (leftY == 0 || rightY == 0) {
//                if (leftY == 0) {stopLeft();}
//                if (rightY == 0) {stopRight();}                
//            } else {
                FL.setPower(leftY);
                BL.setPower(leftY);
                FR.setPower(rightY);
                BR.setPower(rightY);
//            }
            /*
            FL.setPower(sqrt2over2 * (leftX + leftY));
            BL.setPower(sqrt2over2 * (leftX + leftY) * -1);
            FR.setPower(sqrt2over2 * (leftX - leftY));
            BR.setPower(sqrt2over2 * (leftX - leftY) * -1);
            */

            if (gamepad1.left_bumper && !gamepad1.right_bumper) {
                linear.setPower(1);
            } else if (gamepad1.right_bumper && !gamepad1.left_bumper) {
                linear.setPower(-1);
            } else if (!gamepad1.left_bumper && !gamepad1.right_bumper) {
                linear.setPower(0);
            }
            
            if (LTrigger > 0.005 && RTrigger <= 0.005) {
                arm.setPower(LTrigger);
            } else if (RTrigger > 0.005 && LTrigger <= 0.005) {
                arm.setPower(-RTrigger);
            } else if ((LTrigger <= 0.005 && RTrigger <= 0.005) ||
                (LTrigger > 0.005 && RTrigger > 0.005)) {
                arm.setPower(0);
            }
        }
    }
} 
