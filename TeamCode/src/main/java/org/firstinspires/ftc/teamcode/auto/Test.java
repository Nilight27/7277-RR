package org.firstinspires.ftc.teamcode.auto;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanism.Arm;
import org.firstinspires.ftc.teamcode.mechanism.Claw;
@Autonomous
public class Test extends LinearOpMode {
    MecanumDrive drive;

    public static Pose2d initalPose = new Pose2d(new Vector2d(-12, -64), Math.toRadians(90));
    public static Pose2d secondPose = new Pose2d(new Vector2d(-8, -64), Math.toRadians(90));


    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, initalPose);
        Claw claw = new Claw(hardwareMap);
        Arm arm = new Arm(hardwareMap);


        waitForStart();
        TrajectoryActionBuilder tab1 = drive.actionBuilder(initalPose)
                .lineToY(-65)
                .setTangent(0)
                .lineToXConstantHeading(-46)
                .lineToXLinearHeading(-55, Math.PI / 4);




        Actions.runBlocking(new SequentialAction(
                new ParallelAction(
                        tab1.build(),
                        arm.moveTo(1700),
                        claw.moveTo(Claw.Move.OPEN)
                        
                )
        ));
    }
}