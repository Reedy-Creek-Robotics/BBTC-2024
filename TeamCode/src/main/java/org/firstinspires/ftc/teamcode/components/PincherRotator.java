package org.firstinspires.ftc.teamcode.components;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

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
    }

    public Action pincherRotatorLine(){
        return new PincherRotator.PincherRotatorLine();
    }

    public class PincherRotatorTurned implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            pincherRotator.setPosition(.65);
            return false;
        }
    }

    public Action pincherRotatorTurned(){
        return new PincherRotator.PincherRotatorTurned();
    }
}
