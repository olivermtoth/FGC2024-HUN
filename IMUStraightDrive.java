import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="IMU Straight Drive", group="Linear Opmode")
public class IMUStraightDrive extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private BNO055IMU imu = null;
    private Orientation angles;

    @Override
    public void runOpMode() {
        // Initialize the hardware variables.
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        leftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        // Initialize the IMU
        
        

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // Get the initial heading
        float desiredHeading = getHeading();

        // Set motor power
        double basePower = 0.5;

        while (opModeIsActive()) {
            // Get current heading
            float currentHeading = getHeading();

            // Calculate error
            float error = desiredHeading - currentHeading;

            // Apply proportional control
            double correction = error * 0.05;  // Kp value, you may need to tune this

            // Set motor powers, adjusting for the correction
            double leftPower = basePower - correction;
            double rightPower = basePower + correction;

            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);

            telemetry.addData("Desired Heading", desiredHeading);
            telemetry.addData("Current Heading", currentHeading);
            telemetry.addData("Correction", correction);
            telemetry.update();
        }
    }

    // Method to get the current heading from the IMU
    private float getHeading() {
        angles = imu.getAngularOrientation();
        return angles.firstAngle;  // This returns the heading (Z-axis rotation)
    }
}
