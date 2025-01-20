package org.firstinspires.ftc.teamcode.TeleOp.Meets;

import android.os.Build;

import androidx.annotation.NonNull;

// RR imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;

import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;


// Non-RR Imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.rr.MecanumDrive;


@Config
@Autonomous(name = "0+4 LEFT", group = "Autonomous")
public class ZEROPLUSFOURLEFTLM3 extends LinearOpMode {

    public static class Slides {
        private final DcMotorEx slidesL;
        private final DcMotorEx slidesR;

        public Slides(HardwareMap hwMap) {
            slidesL = hwMap.get(DcMotorEx.class, "slidesL");
            slidesR = hwMap.get(DcMotorEx.class, "slidesR");
            slidesL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            slidesR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            slidesL.setDirection(DcMotorSimple.Direction.FORWARD);
            slidesR.setDirection(DcMotorSimple.Direction.REVERSE);
        }


        public class SlidesUp implements Action {
            private boolean slidesInit = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!slidesInit) {
                    slidesL.setPower(1);
                    slidesR.setPower(1);
                    slidesInit = true;
                }

                double posL = slidesL.getCurrentPosition();
                packet.put("slideLPos", posL);

                double posR = slidesR.getCurrentPosition();
                packet.put("slideRPos", posR);

                if (posL < 2250 & posR < 2250) {
                    return true;
                } else {
                    slidesL.setPower(0.05);
                    slidesR.setPower(0.05);
                    return false;
                }
            }
        }

        public Action slidesUp() {
            return new SlidesUp();
        }

        public class SlidesDown implements Action {
            private boolean init = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!init) {
                    slidesL.setPower(-1);
                    slidesR.setPower(-1);
                    init = true;
                }

                double posL = slidesL.getCurrentPosition();
                packet.put("slideLPos", posL);

                double posR = slidesR.getCurrentPosition();
                packet.put("slideRPos", posR);

                if (posL > 50 & posR > 50) {
                    return true;
                } else {
                    slidesL.setPower(0);
                    slidesR.setPower(0);
                    return false;
                }
            }
        }

        public Action slidesDown() {
            return new SlidesDown();
        }
    }

    public static class ExtFront {
        private final Servo backPivot;
        private final Servo frontPivot;
        private final Servo wristClaw;
        private final Servo frontClaw;
        private final Servo leftTransfer;
        private final Servo rightTransfer;

        public ExtFront(HardwareMap hwMap) {
            leftTransfer = hwMap.get(Servo.class, "left");
            rightTransfer = hwMap.get(Servo.class, "right");
            frontClaw = hwMap.get(Servo.class, "frontclaw");
            wristClaw = hwMap.get(Servo.class, "wrist");
            frontPivot = hwMap.get(Servo.class, "frontpivot");
            backPivot = hwMap.get(Servo.class, "backpivot");

            //front claw system
            rightTransfer.setDirection(Servo.Direction.REVERSE);
            frontClaw.setDirection(Servo.Direction.REVERSE);
            frontPivot.setDirection(Servo.Direction.REVERSE);
            backPivot.setDirection(Servo.Direction.REVERSE);
        }


        public class TransferIn implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                leftTransfer.setPosition(.7);
                rightTransfer.setPosition(.7);
                return false;
            }
        }

        public Action transferIn() {
            return new TransferIn();
        }

        public class TransferExtend implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                leftTransfer.setPosition(1);
                rightTransfer.setPosition(1);
                return false;
            }
        }

        public Action transferExtend() {
            return new TransferExtend();
        }

        public class ClawOpen implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                frontClaw.setPosition(0.65);
                return false;
            }
        }

        public Action clawOpen() {
            return new ClawOpen();
        }

        public class ClawClose implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                frontClaw.setPosition(0);
                return false;
            }
        }

        public Action clawClose() {
            return new ClawClose();
        }

        public class WristInit implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                wristClaw.setPosition(0.43);
                return false;
            }
        }

        public Action wristInit() {
            return new WristInit();
        }

        public class WristRotate implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                wristClaw.setPosition(0.8);
                return false;
            }
        }

        public Action wristRotate() {
            return new WristRotate();
        }


        public class FrontPivotGrab implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                frontPivot.setPosition(1.0);
                return false;
            }
        }

        public Action frontPivotGrab() {
            return new FrontPivotGrab();
        }

        public class FrontPivotTransfer implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                frontPivot.setPosition(0.2);
                return false;
            }
        }

        public Action frontPivotTransfer() {
            return new FrontPivotTransfer();
        }

        public class BackPivotBase implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                backPivot.setPosition(0.325);
                return false;
            }
        }


        public Action backPivotBase() {
            return new BackPivotBase();
        }

        public class BackPivotTransfer implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                backPivot.setPosition(0.0);
                return false;
            }
        }

        public Action backPivotTransfer() {
            return new BackPivotTransfer();
        }

    }

    public static class ExtBack {

        private final Servo slidePivot;
        private final Servo slideClaw;

        public ExtBack(HardwareMap hwMap) {
            slidePivot = hwMap.get(Servo.class, "slide pivot");
            slideClaw = hwMap.get(Servo.class, "slide claw");

            slidePivot.setDirection(Servo.Direction.REVERSE);
        }

        public class SlidePivotBase implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                slidePivot.setPosition(0.0);
                return false;
            }
        }

        public Action slidePivotBase() {
            return new SlidePivotBase();
        }

        public class SlidePivotDrop implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                slidePivot.setPosition(0.8);
                return false;
            }
        }

        public Action slidePivotDrop() {
            return new SlidePivotDrop();
        }

        public class SlideClawOpen implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                slideClaw.setPosition(.65);
                return false;
            }
        }

        public Action slideClawOpen() {
            return new SlideClawOpen();
        }

        public class SlideClawClose implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                slideClaw.setPosition(0.41);
                return false;
            }
        }

        public Action slideClawClose() {
            return new SlideClawClose();
        }

    }

    @Override
    public void runOpMode() {


        Pose2d initPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initPose);
        Slides slides = new Slides(hardwareMap);
        ExtFront extFront = new ExtFront(hardwareMap);
        ExtBack extBack = new ExtBack(hardwareMap);


        Actions.runBlocking(
                new ParallelAction(
                        extFront.clawOpen(),
                        extFront.transferIn(),
                        extFront.backPivotBase(),
                        extFront.frontPivotGrab(),
                        extFront.wristInit(),
                        extBack.slidePivotBase(),
                        extBack.slideClawClose(),
                        slides.slidesDown()
                )
        );

        waitForStart();



        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(0,0,0))
                        .stopAndAdd(slides.slidesUp())
                        .stopAndAdd(extBack.slidePivotDrop())
                        .stopAndAdd(extFront.transferExtend())
                        .strafeToLinearHeading(new Vector2d(-23,9), Math.toRadians(45))
                        .stopAndAdd(extBack.slideClawOpen())
                        .waitSeconds(.125)
                        .stopAndAdd(extBack.slidePivotBase())
                        .waitSeconds(.125)
                        //first cycle
                        .stopAndAdd(slides.slidesDown())
                        .strafeToLinearHeading(new Vector2d(-14,16), Math.toRadians(90))
                        .stopAndAdd(extFront.clawClose())
                        .waitSeconds(.25)
                        .stopAndAdd(extFront.backPivotTransfer())
                        .stopAndAdd(extFront.frontPivotTransfer())
                        .stopAndAdd(extFront.transferIn())
                        .waitSeconds(.75)
                        .stopAndAdd(extBack.slideClawClose())
                        .waitSeconds(.1)
                        .stopAndAdd(extFront.clawOpen())
                        .stopAndAdd(extFront.backPivotBase())
                        .stopAndAdd(slides.slidesUp())
                        .stopAndAdd(extBack.slidePivotDrop())
                        .waitSeconds(.25)
                        .strafeToLinearHeading(new Vector2d(-21,8), Math.toRadians(45))
                        //second cycle
                        .stopAndAdd(extFront.transferExtend())
                        .stopAndAdd(extFront.frontPivotGrab())
                        .waitSeconds(.125)
                        .stopAndAdd(extBack.slideClawOpen())
                        .waitSeconds(.125)
                        .stopAndAdd(extBack.slidePivotBase())
                        .waitSeconds(.125)
                        .stopAndAdd(slides.slidesDown())
                        .strafeToLinearHeading(new Vector2d(-25,16), Math.toRadians(90))
                        .stopAndAdd(extFront.clawClose())
                        .waitSeconds(.25)
                        .stopAndAdd(extFront.backPivotTransfer())
                        .stopAndAdd(extFront.frontPivotTransfer())
                        .stopAndAdd(extFront.transferIn())
                        .waitSeconds(.75)
                        .stopAndAdd(extBack.slideClawClose())
                        .waitSeconds(.1)
                        .stopAndAdd(extFront.clawOpen())
                        .stopAndAdd(extFront.backPivotBase())
                        .stopAndAdd(slides.slidesUp())
                        .stopAndAdd(extBack.slidePivotDrop())
                        .waitSeconds(.25)
                        .strafeToLinearHeading(new Vector2d(-23,9), Math.toRadians(45))
                        //third cycle
                        .stopAndAdd(extFront.wristRotate())
                        .stopAndAdd(extFront.frontPivotGrab())
                        .stopAndAdd(extBack.slideClawOpen())
                        .waitSeconds(.125)
                        .stopAndAdd(extBack.slidePivotBase())
                        .waitSeconds(.125)
                        .stopAndAdd(slides.slidesDown())
                        .strafeToLinearHeading(new Vector2d(-12,39), Math.toRadians(180))
                        .stopAndAdd(extFront.transferExtend())
                        .waitSeconds(.75)
                        .stopAndAdd(extFront.clawClose())
                        .waitSeconds(.25)
                        .stopAndAdd(extFront.transferIn())
                        .stopAndAdd(extFront.backPivotTransfer())
                        .stopAndAdd(extFront.frontPivotTransfer())
                        .stopAndAdd(extFront.wristInit())
                        .waitSeconds(.75)
                        .stopAndAdd(extBack.slideClawClose())
                        .waitSeconds(.125)
                        .stopAndAdd(extFront.clawOpen())
                        .stopAndAdd(extFront.backPivotBase())
                        .stopAndAdd(slides.slidesUp())
                        .stopAndAdd(extBack.slidePivotDrop())
                        .strafeToLinearHeading(new Vector2d(-23,9), Math.toRadians(45))
                        .stopAndAdd(extBack.slideClawOpen())
                        .waitSeconds(.125)
                        .stopAndAdd(extBack.slidePivotBase())
                         .strafeToLinearHeading(new Vector2d(0,50), Math.toRadians(90))
                        .stopAndAdd(slides.slidesDown())
                        .strafeToLinearHeading(new Vector2d(18,50), Math.toRadians(90))
                        .build()
        );


        if (isStopRequested()) return;


    }
}