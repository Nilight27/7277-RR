package org.firstinspires.ftc.teamcode.mechanism;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Arm {
    public DcMotor arm, pivot;

    double limit = 1.3;

    public Arm(HardwareMap hardwareMap, HardwareMap hardwareMap1){
        arm = hardwareMap1.get(DcMotor.class, "arm");
        pivot = hardwareMap.get(DcMotor.class, "pivot");
    }

    public void move(double power, double powerA, boolean gamepad,  boolean gamepad2,  boolean gamepad3){
       if(gamepad){
          limit = 1;
        }
       if(gamepad2){
          limit = 1.3;
        }
       if(gamepad3){
          limit = 2;
        }
       pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       pivot.setPower(power);

      arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      arm.setPower(powerA);

    }

}
