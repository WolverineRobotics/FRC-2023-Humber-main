package frc.robot;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class AutoBalancer {
    private DifferentialDrive drive;
    private double pitch;
    private int state;
    private double constant = 6.6;

    private double onChargeDegree;

    public AutoBalancer(DifferentialDrive drive, double pitch){
        this.drive = drive;
        this.pitch = pitch;

        state = 0;
        onChargeDegree = 5;//5
    }

    //states
    /* 0 init
     * 1 too close to drive station
     * 2 too far from drive station
     * 3 level
     */
    public void balancerInit(){
        state = 0;
    }

    public void balancerPeriodic(){

        pitch = Robot.getRobotPitch();
        TestCases();

        //Goldilocks algorithm
        switch(state){
            case 1: //too close
                drive.arcadeDrive(-0.275, 0);//0.275
                break;

            case 2: //too far
                drive.arcadeDrive(0.275, 0);//0.275
                break;
            case 3:

                drive.arcadeDrive(0, 0);
                break;
        }
    }

    void TestCases(){
        if(pitch < -onChargeDegree+constant) {state = 1;} //too close
        else if(pitch > onChargeDegree+constant) {state = 2; } //too far
        else {state = 3;}
    }
    
}