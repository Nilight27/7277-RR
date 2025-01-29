package org.firstinspires.ftc.teamcode.BasicCode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sun.tools.javac.comp.Todo;

/*
* This file contains an example of a Linear "OpMode".
* An OpMode is a 'program' that runs in either the autonomous or the TeleOp period of an FTC match.
* The names of OpModes appear on the menu of the FTC Driver Station.
* When a selection is made from the menu, the corresponding OpMode is executed.
*
* LinearOpMode is as its written and reads the code line by line. In my opinion I feel like the
* formatting of LinearOpMode is much better than OpMode.
*
* This guide is also made in context that you know the basics of java, so keep up.
*/
@Disabled
@TeleOp
public class BasicDriveCode extends LinearOpMode {

    //This is where you call your Motors, Servos, ect.
    // when something is equal to null it means it is equal to nothing
    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotor FL = null;
    private DcMotor BL = null;
    private DcMotor FR = null;
    private DcMotor BR = null;
    double speedLimiter = 1.65;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the Driver Station or Robot Controller devices.
        FL = hardwareMap.get(DcMotor.class, "FL");
        BL = hardwareMap.get(DcMotor.class, "BL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BR = hardwareMap.get(DcMotor.class, "BR");;

        // TODO: Reverse the motors based of how the motors move irl so they are all moving the same direction
        FL.setDirection(DcMotor.Direction.FORWARD);
        BL.setDirection(DcMotor.Direction.FORWARD);
        FR.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.REVERSE);

        //Motor encoders are where the motor counts how many times how a gear move
        //when using encoders always use .setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //This resets the value back to 0 before use
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Now there are 2 modes for encoders. (RUN_WITHOUT_ENCODER) & (RUN_USING_ENCODERS)
        //(RUN_WITHOUT_ENCODERS) denies any access and (RUN_USING_ENCODERS) uses them obv
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //.setZeroPowerBehavior(BRAKE) is great for just making sure the motor absolutely stops when no power is being added
        //REMINDER; In arms or any external mechanism this is absolutely necessary to make sure there is no error when moving/automating your arm
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if (gamepad1.dpad_down) {
                speedLimiter = 2;
            } else if (gamepad1.dpad_up) {
                speedLimiter = 1;
            } else if (gamepad1.dpad_right) {
                speedLimiter = 1.65;
            }

            double max;

            //Note: when trying to use controller commands that are not button presses like joysticks and triggers
            // are supposed to be put into a variable to access the numbers
            double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value because of the (-) sign. When pushing forward usually the value is positive btw
            double lateral = -gamepad1.left_stick_x * 1.1;
            double yaw = -  gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            leftFrontPower /= speedLimiter;
            rightFrontPower /= speedLimiter;
            leftBackPower /= speedLimiter;
            rightBackPower /= speedLimiter;

            // Send calculated power to wheels
            FL.setPower(leftFrontPower);
            FR.setPower(rightFrontPower);
            BL.setPower(leftBackPower);
            BR.setPower(rightBackPower);

            // Show the elapsed game time and wheel power.
            //telemetry.addData() is to spew out numbers to your Driver Station
            //telemetry.update() is used to update any changing values during the period the code is running
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("Current speedLimiter: ", speedLimiter);
            telemetry.update();
        }
    }
}