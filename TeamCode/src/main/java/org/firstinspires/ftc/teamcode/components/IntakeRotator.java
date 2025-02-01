package org.firstinspires.ftc.teamcode.components;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.modules.RunStates;

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
        return new IntakeRotator.IntakeRotatorDefault();
    }

    public class IntakeRotatorPicking implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            intakeRotator.setPosition(RunStates.PICKING.getIntakeRotatorPos());
            return false;
        }
    }

    public Action intakeRotatorPicking(){
        return new IntakeRotator.IntakeRotatorPicking();
    }

    public class IntakeRotatorGrab implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            intakeRotator.setPosition(RunStates.GRAB.getIntakeRotatorPos());
            return false;
        }
    }

    public Action intakeRotatorGrab(){
        return new IntakeRotator.IntakeRotatorGrab();
    }

    public class IntakeRotatorTransfer implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            intakeRotator.setPosition(RunStates.TRANSFER.getIntakeRotatorPos());
            return false;
        }
    }

    public Action intakeRotatorTransfer(){
        return new IntakeRotator.IntakeRotatorTransfer();
    }
}