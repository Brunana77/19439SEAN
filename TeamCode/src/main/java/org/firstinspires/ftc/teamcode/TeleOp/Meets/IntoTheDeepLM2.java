package org.firstinspires.ftc.teamcode.TeleOp.Meets;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanisms.BackLift;
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.mechanisms.FrontExt;

@TeleOp
public class IntoTheDeepLM2 extends OpMode {
    Drivetrain drivetrain = new Drivetrain();
    FrontExt frontExtension = new FrontExt();
    BackLift backLift = new BackLift();

    ElapsedTime runtime;

    @Override
    public void init(){
        drivetrain.init(hardwareMap);
        frontExtension.init(hardwareMap);
        backLift.init(hardwareMap);
        runtime = new ElapsedTime();
    }

    @Override
    public void loop() {

        //P1 drive code, field centric (up is always up)
        float forward = -gamepad1.left_stick_y;
        float right = gamepad1.left_stick_x;
        float turn = gamepad1.right_stick_x;

        double mult;

        if (gamepad1.left_bumper) {
            mult = 1;
        } else {
            mult = 0.4;
        }

        if (gamepad1.options) {
            drivetrain.yawReset();
        }

        double botHeading = drivetrain.yawHeading();

        double rotX = right * Math.cos(-botHeading) - forward * Math.sin(-botHeading);
        double rotY = right * Math.sin(-botHeading) + forward * Math.cos(-botHeading);

        double denim = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(turn), 1);

        drivetrain.fieldCentricDrive(rotX * mult, rotY * mult, turn * mult, denim * mult);

        /**
         * Button Map!
         * P1
         * X - Hard Reset
         * A - Transfer
         * B - Grab
         * L3 - Wrist Turn
         * R3 - Wrist Reset
         * R1 - Front Pivot Down
         * L1 - Turbo
         * Up - Slides Up
         * Right - Slides Mid
         * Down - Slides Down
         * P2
         * Up - Pivot Basket (High)
         * Down - Specimen Drop
         * Left - Specimen Grab
         * Right - Pivot Basket (Middle)
         * B - Reset BackLift
         */

        //Hard reset all positions.
        if (gamepad1.x) {
            backLift.slideClawOpen();
            backLift.slidePivotBase();
            backLift.specimenOpen();
            frontExtension.backPivotBase();
            frontExtension.frontPivotTransfer();
            frontExtension.wristInit();
            frontExtension.frontClawOpen();
            frontExtension.transferInit();
            backLift.slidesBase();
        }

        //Transfer specimen from front to back extension.
        if (gamepad1.a) {
            runtime.reset();
            while (runtime.seconds() <= .75) {
                frontExtension.frontPivotTransfer();
                frontExtension.backPivotTransfer();
                frontExtension.wristInit();
                frontExtension.transferFullIn();
            }
            runtime.reset();
            while (runtime.seconds() <= 0.125) {
                frontExtension.frontClawOpen();
                backLift.slideClawClose();
            }
            frontExtension.frontPivotBase();
            frontExtension.backPivotBase();
        }

        //Lower to grab position
        if (gamepad1.right_bumper){
            frontExtension.frontPivotGrab();
            frontExtension.backPivotGrab();
            frontExtension.frontClawOpen();
        }
        //Grab specimen.
        if (gamepad1.b) {

            runtime.reset();
            while (runtime.seconds() <= 0.25) {
                frontExtension.frontClawGrab();
            }
            frontExtension.frontPivotBase();
            frontExtension.backPivotBase();
        }

        //Turn wrist.
        if (gamepad1.dpad_right) {
            frontExtension.wristRotate();
        }

        if (gamepad1.dpad_left) {
            frontExtension.wristInit();
        }
if(gamepad1.dpad_up){
    frontExtension.wristMiddle();
}

        //Slide positions.
        if (gamepad1.left_stick_button) {
            frontExtension.transferExtend();
            frontExtension.frontPivotBase();
            frontExtension.backPivotBase();
            frontExtension.wristInit();
        }

        if (gamepad1.right_stick_button) {
            frontExtension.transferMiddle();
            frontExtension.frontPivotBase();
            frontExtension.backPivotBase();
            frontExtension.wristInit();
        }

        if (gamepad1.dpad_down) {
            frontExtension.transferFullIn();
            frontExtension.frontPivotBase();
            frontExtension.backPivotBase();
            frontExtension.wristInit();
        }

        //Slide pivot for baskets.
        if (gamepad2.dpad_up) {
            backLift.slidesTop();
            backLift.slidePivotDrop();
        }

        //Drop specimen.
        if (gamepad2.dpad_down) {
            runtime.reset();
            while (runtime.seconds() <= 0.5) {
                backLift.slidesSpecimenHang();
            }
            backLift.specimenOpen();
        }

        //Grab specimen.
        if (gamepad2.dpad_left) {
            runtime.reset();
            while (runtime.seconds() <= 0.5)
                backLift.specimenClose();
        }
            backLift.slidesSpecimenPreHang();
        }

        //Middle slides.
        if (gamepad2.dpad_right) {
            backLift.slidesMiddle();
            backLift.slidePivotDrop();
        }

        //Reset slides and claws.
        if (gamepad2.b) {
        runtime.reset();
        while (runtime.seconds() <= 0.125){
            backLift.slideClawOpen();}
        runtime.reset();
        while (runtime.seconds() <= 0.25) {
            backLift.slidePivotBase();
        }
            backLift.specimenOpen();
            backLift.slidesBase();
        }



    @Override
    public void stop() {
        drivetrain.stopMotors();
    }
}
