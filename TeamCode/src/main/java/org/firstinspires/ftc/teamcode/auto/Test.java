package org.firstinspires.ftc.teamcode.auto;


import static org.firstinspires.ftc.teamcode.mechanism.ArmPIDF.target;

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
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanism.Arm;
import org.firstinspires.ftc.teamcode.mechanism.ArmPIDF;
@Disabled
@Autonomous
public class Test extends LinearOpMode {
    private PIDController controller1, controller2;



    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d initialPose = new Pose2d(-10, -72,Math.PI/2);
        Pose2d initialPose2 = new Pose2d(-10, -72,Math.PI/2);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Pivot pivot = new Pivot(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        ArmPIDF arm = new ArmPIDF(hardwareMap);
        Extender extender = new Extender(hardwareMap);


        // actionBuilder builds from the drive steps passed to it
        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .lineToY(-66)
                .setTangent(0)
                .lineToXLinearHeading(-51, Math.PI/4);


        TrajectoryActionBuilder tab2 = drive.actionBuilder(initialPose2)
                .lineToY(4);





        Action trajectoryActionCloseOut = tab1.endTrajectory().fresh()
//                .setTangent(0)
//                .lineToXLinearHeading(-48, -Math.PI/2)
//                .setTangent(Math.PI/2)
//                .lineToY(-24)
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
                        extender.ExtendUp(),
                        claw.OpenClaw(),
//                        pivot.liftDown(),
//                        trajectoryActionChosen2,
                        trajectoryActionCloseOut

//                        new ParallelAction(
//                                arm.ArmPIDFMove(),
//                                trajectoryActionCloseOut
//                        )
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
                if (pos < 2800) {
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

        public class LiftDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    lift.setPower(-0.5);
                    initialized = true;
                }

                double pos = lift.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos > 600) {
                    return true;
                } else {
                    lift.setPower(0);
                    return false;
                }
            }
        }

        public Action liftDown() {
            return new LiftDown();
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
                claw.setPower(0);
                return false;


            }
        }

        public Action OpenClaw() {
            return new Claw.OpenClaw();
        }
    }

    public class ArmPIDF{
        private PIDController controller1, controller2;

        public double p = 0.05, i = 0, d = 0.001;
        public double f = 0.0005, f2 = 0.01;
        public static final double ticks = 1440;
        private DcMotor lift;
        public int target = 2500;
        public int target2 = 120;

        public ArmPIDF(HardwareMap hardwareMap) {
            lift = hardwareMap.get(DcMotor.class, "pivot");
            controller1 = new PIDController(p, i, d);
        }

        public class ArmPIDFMove implements Action {
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

                controller1.setPID(p,i,d);
                int pivotPos = lift.getCurrentPosition();
                double pid = controller1.calculate(pivotPos, target);
                double ff = Math.cos(Math.toRadians(target/ticks)) * f;

                double power = pid + ff;
                lift.setPower(power);
                packet.put("liftPos", pivotPos);
                if ( pivotPos < 2500) {
                    // true causes the action to rerun
                    return true;
                } else {
                    // false stops action rerun
                    lift.setPower(power);
                    sleep(1000);
                    return false;
                }
                // overall, the action powers the lift until it surpasses
                // 3000 encoder ticks, then powers it off
            }
        }
            public Action ArmPIDFMove() {
                return new ArmPIDFMove();
            }
        public class ArmPIDFMoveDown implements Action {
            // checks if the lift motor has been powered on
            private boolean initialized = false;

            public void move(){
                double target2 = 130;
                controller1.setPID(p,i,d);
                int pivotPos = lift.getCurrentPosition();
                double pid = controller1.calculate(pivotPos, target2);
                double ff = Math.cos(Math.toRadians(target2/ticks)) * f;

                double power = pid + ff;

                lift.setPower(power);
                sleep(5000);

            }

            // actions are formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // powers on motor, if it is not on
                if (!initialized) {
                    lift.setPower(0.8);
                    initialized = true;
                }


                move();
                return false;

            }
        }
        public Action ArmPIDFMoveDown() {
            return new ArmPIDFMoveDown();
        }
    }
    public class Extender{
        private DcMotor extend;

        public Extender(HardwareMap hardwareMap) {
            extend = hardwareMap.get(DcMotor.class, "arm");

        }

        public class ExtendUp implements Action{
            private boolean initialized = false;

            public void move(){
                extend.setPower(-1);
                sleep(1500);
            }
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                // powers on motor, if it is not on
                if (!initialized) {
                    extend.setPower(-1);
                    initialized = true;
                }

                // checks lift's current position
//                double pos = extend.getCurrentPosition();
//                telemetryPacket.put("liftPos", pos);
//                if (pos > -1000) {
//                    // true causes the action to rerun
//                    return true;
//                } else {
//                    // false stops action rerun
//                    extend.setPower(0);
//                    return false;
//                }
                move();
                return false;
                // overall, the action powers the lift until it surpasses
                // 3000 encoder ticks, then powers it off
            }

        }
        public Action ExtendUp(){
            return new ExtendUp();
        }
        public class ExtendDown implements Action{
            private boolean initialized = false;

            public void move(){
                extend.setPower(1);
                sleep(1500);
            }
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                // powers on motor, if it is not on
                if (!initialized) {
                    extend.setPower(1);
                    initialized = true;
                }

                // checks lift's current position
//                double pos = extend.getCurrentPosition();
//                telemetryPacket.put("liftPos", pos);
//                if (pos > -1000) {
//                    // true causes the action to rerun
//                    return true;
//                } else {
//                    // false stops action rerun
//                    extend.setPower(0);
//                    return false;
//                }
                move();
                return false;
                // overall, the action powers the lift until it surpasses
                // 3000 encoder ticks, then powers it off
            }

        }
        public Action ExtendDown(){
            return new ExtendDown();
        }
    }
}

