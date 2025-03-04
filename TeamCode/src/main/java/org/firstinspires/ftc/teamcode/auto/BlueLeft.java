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
        Pose2d initialPose2 = new Pose2d(51, 66,(3*Math.PI)/4);
        Pose2d initialPose3 = new Pose2d(48, 66,(3*Math.PI)/2);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Pivot pivot = new Pivot(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        Extender extender = new Extender(hardwareMap);



        // actionBuilder builds from the drive steps passed to it
        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .lineToY(65)
                .setTangent(0)
                .lineToXLinearHeading(51, (5*Math.PI)/4);


//        TrajectoryActionBuilder tab2 = drive.actionBuilder(initialPose2)
//                .setTangent(0)
//                .lineToXLinearHeading(43, (3*Math.PI)/2)
//                .waitSeconds(2)
//                .setTangent((3*Math.PI))
//                .lineToY(-48);
//
//
//        TrajectoryActionBuilder tab3 = drive.actionBuilder(initialPose3)
//                .lineToY(-43);





        Action trajectoryActionCloseOut = tab1.endTrajectory().fresh()
//                .setTangent(0)
//                .lineToXLinearHeading(-48, -Math.PI/2)
//                .setTangent(Math.PI/2)
//                .lineToY(-24)
                .build();


        waitForStart();
        if (isStopRequested()) return;



        Action trajectoryActionChosen;
        trajectoryActionChosen = tab1.build();

        Actions.runBlocking(
                new SequentialAction(
                        trajectoryActionChosen,
                        pivot.PivotUp(),
                        extender.ExtendUp(),
                        claw.OpenClaw(),
                        extender.ExtendDown(),
                        pivot.liftDown(),
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
                if (pos < 2700) {
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

        public class Coast implements Action{
            private boolean initialized = false;
            public void move(){
                lift.setPower(0.1);
                sleep(100);
                lift.setPower(0);
            }

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized) {
                    lift.setPower(0.1);
                    initialized = true;
                }

                move();
                return false;
            }
        }
        public Action Coast() {
            return new Coast();
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
                claw.setPower(0);
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
                sleep(1000);
                claw.setPower(0);
                return false;


            }
        }

        public Action OpenClaw() {
            return new OpenClaw();
        }
    }
    public class ArmPIDF{
        private PIDController controller1, controller2;

        public double p = 0.05, i = 0, d = 0.001;
        public double f = 0.0005, f2 = 0.01;
        public static final double ticks = 1440;
        private DcMotor lift;
        public int target = 120;
        public int target2 = 200;

        public ArmPIDF(HardwareMap hardwareMap) {
            lift = hardwareMap.get(DcMotor.class, "pivot");
            controller1 = new PIDController(p, i, d);
        }

        public class ArmPIDFMove implements Action {
            // checks if the lift motor has been powered on
            private boolean initialized = false;
            public void move(){
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
                    initialized = true;
                }

                controller1.setPID(p,i,d);
                int pivotPos = lift.getCurrentPosition();
                double pid = controller1.calculate(pivotPos, target2);
                double ff = Math.cos(Math.toRadians(target2/ticks)) * f;

                double power = pid + ff;

                lift.setPower(power);
                sleep(5000);

                return false;

            }
        }
        public Action ArmPIDFMove() {
            return new ArmPIDFMove();
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
                extend.setPower(0);
            }
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                // powers on motor, if it is not on
                if (!initialized) {
                    extend.setPower(-0.8);
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
                sleep(1000);
                extend.setPower(0);
            }

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                if (!initialized) {
                    extend.setPower(0.8);
                    initialized = true;
                }

                move();
                return false;
            }
        }
        public Action ExtendDown(){
            return new ExtendDown();
        }
    }
}