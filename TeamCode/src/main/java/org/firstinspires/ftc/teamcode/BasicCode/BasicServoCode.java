package org.firstinspires.ftc.teamcode.BasicCode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.sun.tools.javac.comp.Todo;


/*
*This class is to teach you how to use the two different types of servos
* A Position Servo and a Continuous Rotation Servo/360 Servo or (CRServo) for short
* the main difference between the two is that a Pos Servo move 180 degrees and is only moved by setting it to a certain Pos
* A CRServo can move as much as it wants and has no limits, it can do a full 360 rotation and continue doing it
* CRServos when coded are basically treated the same as a motor
*/

@Disabled
@TeleOp
public class BasicServoCode extends LinearOpMode {

    // You start like you would any other code and call your class
    private Servo servo;
    /*
    * The only difference between the 2 is that when specifically programming a CRServo you call it instead
    * just calling Servo.
    * if you're having trouble figuring out if your servo is a CRServo or a PosServo make code for a CRServo and test.
    * if you're servo only moves a bit when testing it and cant make a 360 turn then it is just a regular servo
    * Note: there is a board that switches PosServos to CRServos
    */
    private CRServo crServo;

    @Override
    public void runOpMode() throws InterruptedException {
        //initialize the hardware
        servo = hardwareMap.get(Servo.class, "servo");
        crServo = hardwareMap.get(CRServo.class, "crServo");
        /*
          make sure that when you are coding a CRServo to config it to a CRServo otherwise it wont work
         */

        waitForStart();

        // run until the end of the match (driver presses STOP)
        while(opModeIsActive()){
            // TODO: For PosServos make sure to setPosition() to zero before using the servo
            servo.setPosition(0.0);

            //CrServo can basically be coded the same as motors except with
            double power = gamepad1.left_stick_y;
            crServo.setPower(power);

            //Have a statement that stops the servo otherwise it will continue to spin
            if(power == 0){
                crServo.setPower(0.0);
            }

            //Add telemetry Pls I was to lazy to add so make sure u add it yourself

        }

    }
}
