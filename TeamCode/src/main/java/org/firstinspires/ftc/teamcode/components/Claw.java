package org.firstinspires.ftc.teamcode.components;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.modules.Robot;

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