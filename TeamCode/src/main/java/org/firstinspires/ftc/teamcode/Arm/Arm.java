package org.firstinspires.ftc.teamcode.Arm;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm {
    HardwareMap hardwareMap;
    Servo servoWrist;
    Servo servoArmLeft;
    Servo servoArmRight;
    public Arm(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        servoWrist = hardwareMap.get(Servo.class, "servoWrist");
        servoArmLeft = hardwareMap.get(Servo.class, "servoArmLeft");
        servoArmRight = hardwareMap.get(Servo.class, "servoArmRight");
    }

    public enum armState {
        retract,
        extend
    }

    public Action servoArm(armState stateofArm){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket Packet) {
                if (stateofArm == armState.retract) {
                    servoArmLeft.setPosition(0.5);
                    servoArmRight.setPosition(0.5);
                    servoWrist.setPosition(-0.5);

                }
                if (stateofArm == armState.extend) {
                    servoArmLeft.setPosition(-1);
                    servoArmLeft.setPosition(-1);
                    servoWrist.setPosition(0);
                }
                return false;
            }
        };
    }
}