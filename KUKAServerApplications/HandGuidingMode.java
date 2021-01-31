package KUKAServerApplications;

import com.kuka.connectivity.motionModel.smartServo.ISmartServoRuntime;
import com.kuka.connectivity.motionModel.smartServo.ServoMotion;
import com.kuka.connectivity.motionModel.smartServo.SmartServo;
import com.kuka.med.deviceModel.LBRMed;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.motionModel.controlModeModel.JointImpedanceControlMode;
import com.kuka.task.ITaskLogger;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;

public class HandGuidingMode
{
    private LBRMed LBR_Med_;
    private ITaskLogger logger_;

    public HandGuidingMode(LBRMed LBR_Med,ITaskLogger logger)
    {
        LBR_Med_ = LBR_Med;
        logger_ = logger;
    }

    public void start()
    { 
        KUKAServerManager.handguiding_endflag_.compareAndSet(true, false);
        LBR_Med_.move(ptp(LBR_Med_.getCurrentJointPosition()));
        if (!ServoMotion.validateForImpedanceMode(LBR_Med_))
        {
            //getLogger().info("Validation of Torque Model failed");
        } 
        JointImpedanceControlMode controlMode_1 = new JointImpedanceControlMode(
                2000.0, 2000.0, 2000.0, 2000.0, 100.0, 100.0, 100.0);
        controlMode_1.setStiffness(1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0); // 僵硬度
        controlMode_1.setDampingForAllJoints(0.1); // 阻尼
        controlMode_1.setMaxJointSpeed(1000, 1000, 1000, 1000, 1000, 1000, 1000);
       
        JointImpedanceControlMode controlMode_2 = new JointImpedanceControlMode(
                2000.0, 2000.0, 2000.0, 2000.0, 100.0, 100.0, 100.0);
        controlMode_1.setStiffness(1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0);
        controlMode_1.setDampingForAllJoints(0.1);
        controlMode_1.setMaxJointSpeed(1000, 1000, 1000, 1000, 1000, 1000, 1000);

        JointPosition JntPose = LBR_Med_.getCurrentJointPosition();
        SmartServo servoMotion = new SmartServo(JntPose);
        servoMotion.setMinimumTrajectoryExecutionTime(40e-3);

        LBR_Med_.moveAsync(servoMotion.setMode(controlMode_1));
        ISmartServoRuntime servoMotionRuntime = servoMotion.getRuntime();
        servoMotionRuntime.updateWithRealtimeSystem();
       
        while (KUKAServerManager.handguiding_endflag_.get() == false)      		
        {
//        	logger_.info(String.valueOf(KUKAServerManager.handguiding_endflag_.get()));
            JointPosition curJntPose = LBR_Med_.getCurrentJointPosition();

            Boolean a1 = -2.8 < curJntPose.get(0) && curJntPose.get(0) < 2.8;
            Boolean a2 = -1.9 < curJntPose.get(1) && curJntPose.get(1) < 1.9;
            Boolean a3 = -2.8 < curJntPose.get(2) && curJntPose.get(2) < 2.8;
            Boolean a4 = -1.9 < curJntPose.get(3) && curJntPose.get(3) < 1.9;
            Boolean a5 = -2.8 < curJntPose.get(4) && curJntPose.get(4) < 2.8;
            Boolean a6 = -1.9 < curJntPose.get(5) && curJntPose.get(5) < 1.9;
            Boolean a7 = -2.8 < curJntPose.get(6) && curJntPose.get(6) < 2.8;

            boolean limit = a1 && a2 && a3 && a4 && a5 && a6 && a7;
            if(limit)
            {
                servoMotionRuntime.changeControlModeSettings(controlMode_1);
                servoMotionRuntime.setDestination(curJntPose);
            }
            else
            {
                servoMotionRuntime.changeControlModeSettings(controlMode_2);
            }
         }
        logger_.info("3");
        servoMotionRuntime.stopMotion();
        logger_.info("stop");
        JointPosition ptpPose = LBR_Med_.getCurrentJointPosition();
        LBR_Med_.move(ptp(ptpPose).setJointVelocityRel(0.4));

    }
}

