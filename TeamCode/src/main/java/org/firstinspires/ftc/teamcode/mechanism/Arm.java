package org.firstinspires.ftc.teamcode.mechanism;

import static android.os.SystemClock.sleep;

import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class Arm {

    public DcMotor arm, pivot;
    public VoltageSensor voltageSensor;
    public PIDController controller1, controller2;

    public static double p = 0.05, i = 0, d = 0.001;
    public static double p2 = 0.029, i2 = 0, d2 = 0;
    public static double f = 0.0005, f2 = 0.01;



    public static final double ticks = 1440;
    public static final double ticks2 = 960;
    double limit = 1;

    public Arm(HardwareMap hardwareMap) {
        arm = hardwareMap.get(DcMotor.class, "arm");
        pivot = hardwareMap.get(DcMotor.class, "pivot");
        controller1 = new PIDController(p, i, d);

    }

    public void move(double power, double powerA, boolean gamepad, boolean gamepad2, boolean gamepad3) {
        if (gamepad) {
            limit = 1;
        }
        if (gamepad2) {
            limit = 1.3;
        }
        if (gamepad3) {
            limit = 2;
        }
        pivot.setPower(power / limit);

        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setPower(powerA);

    }

    public Action moveTo(int target) {

        controller1.setPID(p, i, d);
        int pivotPos = pivot.getCurrentPosition();
        double pid = controller1.calculate(pivotPos, target);
        double ff = Math.cos(Math.toRadians(target / ticks)) * f;

        double power = pid + ff;
        pivot.setPower(power);


        return null;
    }

    public void autoMove(double power, int time){
        pivot.setPower(power);
        sleep(time);
    }
    public void lift(){
        autoMove(-1, 3000);
        autoMove(-0.3,10000);
    }

}
