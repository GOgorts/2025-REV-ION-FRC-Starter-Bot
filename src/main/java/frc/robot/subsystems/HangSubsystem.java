package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HangSubsystemConstants;;

public class HangSubsystem extends SubsystemBase{

    private SparkMax hangMotor = new SparkMax(HangSubsystemConstants.kHangMotorCanId, MotorType.kBrushless);

    private boolean holdWhenIdle = false;

    public HangSubsystem() {}

    public void setHangPower(double power){
        hangMotor.set(power);
    }

    // Assuming forward means hang
    public Command runHangCommand() {
        return this.run(() -> {
            holdWhenIdle = true;
            setHangPower(HangSubsystemConstants.kForward);
        });
    }

    // Assuming forward means unhang
    public Command reverseHangCommand() {
        return this.run(() -> {
            holdWhenIdle = false;
            setHangPower(HangSubsystemConstants.kReverse);
        });
    }

    /*  
     *  Command to run when no button is being pressed. If in the process of hanging, run motor at a lesser power
     *  so the robot will not let go.
     */
    public Command idleCommand() {
        return this.run(() -> {
            if (holdWhenIdle) 
                setHangPower(HangSubsystemConstants.kHold);
            else 
                setHangPower(0.0);
            });
  }
    
}
