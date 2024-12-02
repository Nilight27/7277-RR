package org.firstinspires.ftc.teamcode.mechanism;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.mechanism.constants.ClawConstants;

public class Claw {

    public CRServo claw;
    public ClawConstants CC;
    public enum Action{
            OPEN,
            CLOSE,
            REG
    }

    public  Claw(HardwareMap hardwareMap){
        claw = hardwareMap.get(CRServo.class,"claw");
    }
    public void move(double power) {
        claw.setPower(power);

        if(power == 0){
            claw.setPower(0);
        }

    }
}
