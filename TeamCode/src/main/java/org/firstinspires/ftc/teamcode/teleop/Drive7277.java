package org.firstinspires.ftc.teamcode.teleop;

import android.graphics.Path;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanism.Arm;
import org.firstinspires.ftc.teamcode.mechanism.Claw;
import org.firstinspires.ftc.teamcode.mechanism.DriveTrain;

@TeleOp(name = "Drive7277")
public class Drive7277 extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();
    private Claw claw;
    boolean check;
    double speedLimiter = 1;
    double limit = 1.65;
    public Boolean movement;
    public PIDController controller1;

    public static double p = 0.05, i = 0, d = 0;
    public static double p2 = 0.029, i2 = 0, d2 = 0;
    public static double f = 0.00005, f2 = 0.01;
    public static int target = 0, target2 = 0;
    public static final double ticks = 1440;
    public int change = 0;



    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        DriveTrain driveTrain = new DriveTrain(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        Arm arm = new Arm(hardwareMap);
        controller1 = new PIDController(p,i,d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            driveTrain.calculations(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.dpad_up, gamepad1.dpad_right, gamepad1.dpad_down);

            double rotate = gamepad2.right_stick_x;
            claw.move(rotate);


            if (gamepad2.left_bumper){
                change = 0;
            }if (gamepad2.right_bumper){
                change = 1;
            }

            if (change == 0){
                arm.move(gamepad2.left_stick_y, gamepad2.left_stick_x, gamepad2.dpad_up, gamepad2.dpad_right, gamepad2.dpad_down);
            }

            if (change == 1){
                
                if (gamepad2.a){
                    target = 100;
                }if (gamepad2.b){
                    target = 120;
                }if (gamepad2.x){
                    target = 150;
                }if (gamepad2.y){
                    target = 135;
                }

                controller1.setPID(p,i,d);
                int pivotPos = arm.pivot.getCurrentPosition();
                double pid = controller1.calculate(pivotPos, target);
                double ff = Math.cos(Math.toRadians(target/ticks)) * f;

                double power = pid + ff;
                arm.pivot.setPower(power);
            }
            if(gamepad2.a){
                arm.lift();
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Open/Close", check);
            telemetry.addData("Change", change);
            telemetry.addData("Motor Pos", arm.pivot.getCurrentPosition());
            telemetry.update();
        }
    }
}