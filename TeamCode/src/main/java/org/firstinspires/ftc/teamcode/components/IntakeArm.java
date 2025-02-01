package org.firstinspires.ftc.teamcode.components;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.modules.RunStates;

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
        return new IntakeArm.IntakeArmDefault();
    }

    public class IntakeArmGrab implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            intakeArm.setPosition(RunStates.GRAB.getArmPos());
            return false;
        }
    }

    public Action intakeArmGrab(){
        return new IntakeArm.IntakeArmGrab();
    }

    public class IntakeArmTransfer implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            intakeArm.setPosition(RunStates.TRANSFER.getArmPos());
            return false;
        }
    }

    public Action intakeArmTransfer(){
        return new IntakeArm.IntakeArmTransfer();
    }
}