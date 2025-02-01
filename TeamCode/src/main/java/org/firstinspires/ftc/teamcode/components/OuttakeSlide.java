package org.firstinspires.ftc.teamcode.components;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.modules.Robot;

public class OuttakeSlide {
        private DcMotor outtakeSlideLeft;
        private DcMotor outtakeSlideRight;

        public OuttakeSlide(HardwareMap hardwareMap){
            outtakeSlideRight = hardwareMap.get(DcMotor.class, "outtakeSlideRight");
            outtakeSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            outtakeSlideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            outtakeSlideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            outtakeSlideRight.setDirection(DcMotor.Direction.FORWARD);

            outtakeSlideLeft = hardwareMap.get(DcMotor.class, "outtakeSlideLeft");
            outtakeSlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            outtakeSlideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            outtakeSlideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            outtakeSlideLeft.setDirection(DcMotor.Direction.REVERSE);
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
            return new OuttakeSlide.OuttakeSlideUp();
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

                if(pos > Robot.OUTTAKE_SLIDE_DOWN){
                    return true;
                } else {
                    outtakeSlideLeft.setPower(0);
                    outtakeSlideRight.setPower(0);
                    return false;
                }
            }
        }

        public Action outtakeSlideDown(){
            return new OuttakeSlide.OuttakeSlideDown();
        }

    }