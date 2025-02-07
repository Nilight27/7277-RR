package org.firstinspires.ftc.teamcode.auto;


import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous
public class RedRight extends LinearOpMode {
    private PIDController controller1, controller2;



    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d initialPose = new Pose2d(10, -72,Math.PI/2);
        Pose2d initialPose2 = new Pose2d(10, -65,Math.PI/2);
        Pose2d initialPose3 = new Pose2d(10, -43,Math.PI/2);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Pivot pivot = new Pivot(hardwareMap);
        Claw claw = new Claw(hardwareMap);


        // actionBuilder builds from the drive steps passed to it
        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .lineToY(-65);

        TrajectoryActionBuilder tab2 = drive.actionBuilder(initialPose2)
                .lineToY(-43);

        TrajectoryActionBuilder tab3 = drive.actionBuilder(initialPose3)
                .lineToY(-45);



        Action trajectoryActionCloseOut = tab1.endTrajectory().fresh()
                .setTangent(0)
                .lineToX(60)
                .build();


        waitForStart();
        if (isStopRequested()) return;



        Action trajectoryActionChosen;
        Action trajectoryActionChosen2;
        trajectoryActionChosen = tab1.build();
        trajectoryActionChosen2 = tab2.build();

        Actions.runBlocking(
                new SequentialAction(
                        trajectoryActionChosen,
                        pivot.PivotUp(),
                        trajectoryActionChosen2,
                        claw.CloseClaw(),
                        pivot.PivotDown(),
                        trajectoryActionCloseOut
                )
        );
    }
    public class Pivot {
        private PIDController controller1, controller2;

        public double p = 0.05, i = 0, d = 0.001;
        public double f = 0.0005, f2 = 0.01;
        public static final double ticks = 1440;
        private DcMotor lift;
        public int target = 2000;

        public Pivot(HardwareMap hardwareMap) {
            lift = hardwareMap.get(DcMotor.class, "pivot");
            controller1 = new PIDController(p, i, d);
        }

        public class PivotUp implements Action {
            // checks if the lift motor has been powered on
            private boolean initialized = false;

            // actions are formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // powers on motor, if it is not on
                if (!initialized) {
                    lift.setPower(0.8);
                    initialized = true;
                }

                // checks lift's current position
                double pos = lift.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < 2500) {
                    // true causes the action to rerun
                    return true;
                } else {
                    // false stops action rerun
                    lift.setPower(0);
                    return false;
                }
                // overall, the action powers the lift until it surpasses
                // 3000 encoder ticks, then powers it off
            }
        }

        public Action PivotUp() {
            return new PivotUp();
        }

        public class PivotDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    lift.setPower(-0.8);
                    initialized = true;
                }

                double pos = lift.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos > 1700) {
                    return true;
                } else {
                    lift.setPower(0);
                    return false;
                }
            }
        }

        public Action PivotDown() {
            return new PivotDown();
        }
        public class PivotDown2 implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    lift.setPower(-0.3);
                    initialized = true;
                }

                double pos = lift.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos > 1000) {
                    return true;
                } else {
                    lift.setPower(0);
                    return false;
                }
            }
        }
        public Action PivotDown2() {
            return new PivotDown2();
        }

    }
    public class Claw {
        private CRServo claw;

        public Claw(HardwareMap hardwareMap) {
            claw = hardwareMap.get(CRServo.class, "claw");
        }

        public class CloseClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPower(1.0);
                sleep(2000);
                return false;
            }
        }

        public Action CloseClaw() {
            return new CloseClaw();
        }

        public class OpenClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPower(-1);
                sleep(2000);
                return false;
            }
        }

        public Action OpenClaw() {
            return new OpenClaw();
        }
    }
}
