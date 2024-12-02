package org.firstinspires.ftc.teamcode.mechanism;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
@TeleOp
public class ArmPIDF extends LinearOpMode {
    private PIDController controller1, controller2;
    public static double p = 0, i = 0, d = 0;
    public static double p2 = 0, i2 = 0, d2 = 0;
    public static double f = 0, f2 = 2;
    public static int target = 0, target2 = 0;


    public static final double ticks = 960;

    @Override
    public void runOpMode() throws InterruptedException {
        Arm arm = new Arm(hardwareMap,hardwareMap);
        controller1 = new PIDController(p,i,d);
        controller2 = new PIDController(p2,i2,d2);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());



        waitForStart();
        while(opModeIsActive()){
            //The Pivot motor PID that is being tuned
            controller1.setPID(p,i,d);
            int pivotPos = arm.pivot.getCurrentPosition();
            double pid = controller1.calculate(pivotPos, target);
            double ff = Math.cos(Math.toRadians(target/ticks)) * f;

            double power = pid + ff;
            arm.pivot.setPower(power);


            //The Extend Motor PID That is being tuned
            controller2.setPID(p2,i2,d2);
            int extendPos = arm.arm.getCurrentPosition();
            double pid2 = controller2.calculate(extendPos, target2);
            double ff2 = Math.cos(Math.toRadians(target2/ticks)) * f2;

            double power2 = pid2 + ff2;
            arm.arm.setPower(power2);

            //Telemetry
            telemetry.addData("Pivot Pos", pivotPos);
            telemetry.addData("Target", target);
            telemetry.update();
        }
    }
}
