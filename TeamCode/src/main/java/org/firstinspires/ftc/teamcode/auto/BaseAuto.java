package org.firstinspires.ftc.teamcode.auto;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

// Non-RR imports
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.modules.Robot;
import org.firstinspires.ftc.teamcode.modules.RunStates;

import com.qualcomm.robotcore.hardware.DcMotor;

public class BaseAuto {

    public class OuttakeSlide {
        private DcMotor outtakeSlideLeft;
        private DcMotor outtakeSlideRight;

        public OuttakeSlide(HardwareMap hardwareMap){
            outtakeSlideRight = hardwareMap.get(DcMotor.class, "outtakeSlideRight");
            outtakeSlideRight.setMode(RUN_USING_ENCODER);
            outtakeSlideRight.setZeroPowerBehavior(BRAKE);
            outtakeSlideLeft.setDirection(FORWARD);

            outtakeSlideLeft = hardwareMap.get(DcMotor.class, "outtakeSlideLeft");
            outtakeSlideLeft.setMode(RUN_USING_ENCODER);
            outtakeSlideLeft.setZeroPowerBehavior(BRAKE);
            outtakeSlideLeft.setDirection(REVERSE);
        }

        public class OuttakeSlideUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                    if(!initialized){
                         outtakeSlideLeft.setPower(1);
                         outtakeSlideRight.setPower(1);
                         initialized = true;
                    }

                    double pos = (outtakeSlideLeft.getCurrentPosition() + outtakeSlideRight.getCurrentPosition()) / 2;
                    packet.put("Outtake Slide Pos", pos);

                    if(pos < Robot.OUTTAKE_SLIDE_UP){
                        return true;
                    } else {
                        outtakeSlideLeft.setPower(0);
                        outtakeSlideRight.setPower(0);
                        return false;
                    }
            }
        }

        public Action outtakeSlideUp(){
            return new OuttakeSlideUp();
        }

        public class OuttakeSlideDown implements Action{
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                if(!initialized){
                    outtakeSlideLeft.setPower(-1);
                    outtakeSlideRight.setPower(-1);
                    initialized = true;
                }

                double pos = (outtakeSlideLeft.getCurrentPosition() + outtakeSlideRight.getCurrentPosition()) / 2;
                packet.put("Outtake Slide Pos", pos);

                if(pos < Robot.OUTTAKE_SLIDE_DOWN){
                    return true;
                } else {
                    outtakeSlideLeft.setPower(0);
                    outtakeSlideRight.setPower(0);
                    return false;
                }
            }
        }

        public Action outtakeSlideDown(){
            return new OuttakeSlideDown();
        }

    }

    public class Basket {
        private Servo basket;

        public Basket(HardwareMap hardwareMap){
            basket = hardwareMap.get(Servo.class, "basket");
        }

            public class BasketUp implements Action {

                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    basket.setPosition(Robot.BASKET_UP);
                    return false;
                }
            }
            
            public Action basketUp(){
                return new BasketUp();
            }

        public class BasketDown implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                basket.setPosition(Robot.BASKET_DOWN);
                return false;
            }
        }

        public Action basketDown(){
            return new BasketDown();
        }
    }
    
    public class Pincher {
        private Servo pincher;
        
        public Pincher(HardwareMap hardwareMap){
               pincher = hardwareMap.get(Servo.class, "pincher");
        }
        
            public class PincherOpen implements Action {
     
               @Override
               public boolean run(@NonNull TelemetryPacket packet) {
                     pincher.setPosition(Robot.PINCHER_OPEN);
                     return false;
               }
            }
            
            public Action pincherOpen(){
                return new PincherOpen();
            }

        public class PincherClose implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                pincher.setPosition(Robot.PINCHER_CLOSED);
                return false;
            }
        }

        public Action pincherClose(){
            return new PincherClose();
        }
    }

    public class Claw {
        private Servo claw;

        public Claw(HardwareMap hardwareMap){
            claw = hardwareMap.get(Servo.class, "claw");
        }

        public class ClawOpen implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(Robot.CLAW_OPEN);
                return false;
            }
        }

        public Action clawOpen(){
            return new ClawOpen();
        }

        public class ClawClose implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(Robot.CLAW_CLOSED);
                return false;
            }
        }

        public Action clawClose(){
               return new ClawClose();
        }
    }
    
    public class IntakeSlide{
        private Servo intakeSlide;
        
        public IntakeSlide(HardwareMap hardwareMap){
            intakeSlide = hardwareMap.get(Servo.class, "intakeSlide");
        }

        public class IntakeSlideIn implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intakeSlide.setPosition(Robot.INTAKE_SLIDE_IN);
                return false;
            }
        }

        public Action intakeSlideIn(){
            return new IntakeSlideIn();
        }

        public class IntakeSlideOut implements Action{
               @Override
               public boolean run(@NonNull TelemetryPacket packet) {
                    intakeSlide.setPosition(Robot.INTAKE_SLIDE_OUT);
                    return false;
               }
        }

        public Action intakeSlideOut(){
            return new IntakeSlideOut();
        }
    }

    public class IntakeArm{
        private Servo intakeArm;

        public IntakeArm(HardwareMap hardwareMap){
            intakeArm = hardwareMap.get(Servo.class, "intakeArm");
        }

        public class IntakeArmDefault implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intakeArm.setPosition(RunStates.DEFAULT.getArmPos());
                return false;
            }
        }

            public Action intakeArmDefault(){
               return new IntakeArmDefault();
            }

            public class IntakeArmGrab implements Action {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    intakeArm.setPosition(RunStates.GRAB.getArmPos());
                    return false;
                }
            }

            public Action intakeArmGrab(){
                return new IntakeArmGrab();
            }

            public class IntakeArmTransfer implements Action {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    intakeArm.setPosition(RunStates.TRANSFER.getArmPos());
                    return false;
                }
            }

            public Action intakeArmTransfer(){
                return new IntakeArmTransfer();
            }
    }

    public class PincherRotator{
        private Servo pincherRotator;

        public PincherRotator(HardwareMap hardwareMap){
            pincherRotator = hardwareMap.get(Servo.class, "pincherRotator");
        }

        public class PincherRotatorLine implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                pincherRotator.setPosition(.35);
                return false;
            }

            public Action pincherRotatorLine(){
                return new PincherRotatorLine();
            }

            public class PincherRotatorTurned implements Action {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    pincherRotator.setPosition(.65);
                    return false;
                }
            }

            public Action pincherRotatorTurned(){
                return new PincherRotatorTurned();
            }
        }
    }

    public class IntakeRotator{
        private Servo intakeRotator;

        public IntakeRotator(HardwareMap hardwareMap){
            intakeRotator = hardwareMap.get(Servo.class, "intakeRotator");
        }

        public class IntakeRotatorDefault implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intakeRotator.setPosition(RunStates.DEFAULT.getIntakeRotatorPos());
                return false;
            }
        }

        public Action intakeRotatorDefault(){
            return new IntakeRotatorDefault();
        }

        public class IntakeRotatorPicking implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intakeRotator.setPosition(RunStates.PICKING.getIntakeRotatorPos());
                return false;
            }
        }

        public Action intakeRotatorPicking(){
            return new IntakeRotatorPicking();
        }

        public class IntakeRotatorGrab implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intakeRotator.setPosition(RunStates.GRAB.getIntakeRotatorPos());
                return false;
            }
        }

        public Action intakeRotatorGrab(){
            return new IntakeRotatorGrab();
        }

        public class IntakeRotatorTransfer implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intakeRotator.setPosition(RunStates.TRANSFER.getIntakeRotatorPos());
                return false;
            }
        }

        public Action intakeRotatorTransfer(){
            return new IntakeRotatorTransfer();
        }
    }
}
