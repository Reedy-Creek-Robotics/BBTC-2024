package org.firstinspires.ftc.teamcode.components;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.modules.Robot;

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
        return new IntakeSlide.IntakeSlideIn();
    }

    public class IntakeSlideOut implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            intakeSlide.setPosition(Robot.INTAKE_SLIDE_OUT);
            return false;
        }
    }

    public Action intakeSlideOut(){
        return new IntakeSlide.IntakeSlideOut();
    }
}
