package frc.robot.subsystems;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Pneumatics;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class Claw extends SubsystemBase{

    //create Motor/Solenoid Objects

    private DoubleSolenoid pitchSolenoid;
    


    public Claw(){
        //initalize Solenoid / Motors
        pitchSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Pneumatics.kDoubleSolenoidLeftSlot, Pneumatics.kDoubleSolenoidRightSlot);
        
        //------Double Solenoid setup------
        //initalize the solenoid to start in the Forward Position
        pitchSolenoid.set(DoubleSolenoid.Value.kForward);
    }
    /**
    * enables the Pneumatic Piston to extend
    * @param extend extends the pneumatic if true
    */
    public void extendPneumatic(boolean extend){
        if(extend)
        {
            pitchSolenoid.set(kForward);
            return;
        }
        pitchSolenoid.set(kReverse);
    }
}


