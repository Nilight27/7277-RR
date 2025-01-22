package org.firstinspires.ftc.teamcode.auto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.MecanumDrive;
@Autonomous
public class BlueLeft extends LinearOpMode {
    private PIDController controller1, controller2;



    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d initialPose = new Pose2d(10, 72,(3*Math.PI)/2);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Pivot pivot = new Pivot(hardwareMap);
        Claw claw = new Claw(hardwareMap);


        // actionBuilder builds from the drive steps passed to it
        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .lineToY(65)
                .setTangent(0)
                .lineToXConstantHeading(46)
                .lineToXLinearHeading(55, (5*Math.PI)/4);



        Action trajectoryActionCloseOut = tab1.endTrajectory().fresh()
//                .setTangent(0)
//                .lineToXLinearHeading(-36, -Math.PI/4)
//                .setTangent(Math.PI/2)
//                .lineToY(0)
                .build();


        waitForStart();
        if (isStopRequested()) return;



        Action trajectoryActionChosen;
        trajectoryActionChosen = tab1.build();

        Actions.runBlocking(
                new SequentialAction(
                        trajectoryActionChosen,
                        pivot.PivotUp(),
                        claw.OpenClaw(),
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
        public int target = 1500;

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
            return new Pivot.PivotUp();
        }

        public class LiftDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    lift.setPower(-0.8);
                    initialized = true;
                }

                double pos = lift.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos > 200) {
                    return true;
                } else {
                    lift.setPower(0);
                    return false;
                }
            }
        }

        public Action liftDown() {
            return new Pivot.LiftDown();
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
            return new Claw.CloseClaw();
        }

        public class OpenClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPower(-1);
                sleep(1000);
                return false;
            }
        }

        public Action OpenClaw() {
            return new Claw.OpenClaw();
        }
    }
}
