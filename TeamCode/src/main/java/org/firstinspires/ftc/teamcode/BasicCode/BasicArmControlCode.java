package org.firstinspires.ftc.teamcode.BasicCode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.sun.tools.javac.comp.Todo;

/*
    * This file will show you the 2 different ways to move a arm with a joystick, the same can also be applied with the back triggers
    * the code will have parts that show which parts are regular power or set positions
    * P.S: This builds off of the basics explained in the "BasicDriveCode" file.
*/
@Disabled
public class BasicArmControlCode extends LinearOpMode {

    //Call DcMotors
    private DcMotor arm;
    private DcMotor armPos;

    double speedLimiter = 1;


    @Override
    public void runOpMode() throws InterruptedException {
        //Initialize via hardware map
        arm = hardwareMap.get(DcMotor.class, "arm");

        armPos = hardwareMap.get(DcMotor.class, "armPos");

        //armPos Init starts here

        //Stop and reset will rest any encoder value to 0
        armPos.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //run to position will basically count the ticks from the motor and when it reaches its desired position it stops adding power to the motor
        armPos.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //armPos Init ends here


        while(opModeIsActive()){

            //Making the arm code that you will use starts here:

            //First you contain the values that you get from either your trigger or joysticks in a variable
            double power = gamepad2.left_stick_y;

            //add a speed limiter just in case to make it easier for your drivers
            //a speed limiter of 1 will be the fastest and anything higher than that will decrease the amount of power your motor gets
            //TODO Change values to make what you need
            if (gamepad2.dpad_down) {
                speedLimiter = 2;
            } else if (gamepad2.dpad_up) {
                speedLimiter = 1;
            } else if (gamepad2.dpad_right) {
                speedLimiter = 1.65;
            }

            //Take the variable of your speed limiter and divide power with it
            double truePower = power/speedLimiter;
            arm.setPower(truePower);

            //add telemtry to see the power of your motor and your current speed limit
            telemetry.addData("Arm Speed", truePower);
            telemetry.addData("Limiter", speedLimiter);
            telemetry.update();


        }

    }
}
