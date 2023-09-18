package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

import java.lang.Math;

public class SpeedLerp {
    private double initial = 0;
    private double current = 0;
    private double target = 0;
    private double difference = 0;
    private double speed = 0;
    private double threshold = 0.01;
    private double kP = 0.02; //0.03 for balancer

    private MotorControllerGroup motor;
    private RelativeEncoder encoder;

    public SpeedLerp(CANSparkMax motor, RelativeEncoder encoder, double target, double threshold){
        
        this.target = target;

        this.motor = new MotorControllerGroup(motor);
        this.threshold = threshold;

        this.encoder = motor.getEncoder();
        this.difference = target - current;


    }

    public SpeedLerp(MotorControllerGroup motor, RelativeEncoder encoder, double target, double threshold){
        
        this.target = target;

        this.motor = motor;
        this.threshold = 0.05;

        this.encoder = encoder;
        this.difference = target - current;


    }

    public void lerpPeriodic() {
        current = encoder.getPosition();
        this.difference = target - current;

        motor.set(difference*kP);
    }

    public boolean isFinished(){
        if(Math.abs(target - current) <= threshold){
            return true;
        }
        else{
            return false;
        }
    }

    public void setTarget(double target) {
        this.target = target;
    }

    public void setThresh(double threshold) {
        this.threshold = threshold;
    }
}