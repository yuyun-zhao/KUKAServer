package KUKAServerApplications;

import com.kuka.common.ThreadUtil;
import com.kuka.connectivity.motionModel.directServo.DirectServo;
import com.kuka.connectivity.motionModel.directServo.IDirectServoRuntime;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.task.ITaskLogger;

public class DirectServoMode implements Runnable
{
	private LBR _lbr;
    private Tool _toolAttachedToLBR;
	private ITaskLogger logger_;

    private static final int MILLI_SLEEP_TO_EMULATE_COMPUTATIONAL_EFFORT = 30;
    
    DirectServoMode(LBR lbr,Tool tool,ITaskLogger logger)
    {
        _lbr = lbr;
        _toolAttachedToLBR = tool;
        logger_=logger;
    }

    /**
     * Calculate the distance between two point.
     * @param aFrame
     * @param pos
     * @return
     */
    public double cartesianDistance(Frame aFrame, double pos[])
    {
    	double dst = Math.sqrt(Math.pow(aFrame.getX() - pos[0], 2) +
				  Math.pow(aFrame.getY() - pos[1], 2) + Math.pow(aFrame.getZ() - pos[2], 2));
    	
    	return dst;
    }
    
    /**
     * Calculate the angle of two vector.
     * @param aFrame
     * @param pos
     * @return
     */
    public double angleDistance(Frame aFrame, double pos[])
    {
    	double angle = Math.acos((aFrame.getX() * pos[0] + aFrame.getY() * pos[1] + aFrame.getZ() * pos[2]) / 
    			    	((Math.sqrt(aFrame.getX() * aFrame.getX() + aFrame.getY() * aFrame.getY() + aFrame.getZ() * aFrame.getZ())) * 
    			    		(Math.sqrt(pos[0] * pos[0] + pos[1] * pos[1] + pos[2] * pos[2]))));
    	
    	return angle;
    }

	public void start()
	{
		Thread t_background = new Thread(this);
		t_background.setDaemon(true); // 设置为守护线程
		t_background.start();
	}
    
	@Override
	public void run() {
		try{
			while(KUKAServerManager.servoThread_ == false){
				ThreadUtil.milliSleep(100);
			}
		}catch(Exception e ){
			logger_.error(e.toString());
		}
		// TODO Auto-generated method stub
		boolean doDebugPrints = false;
        
        DirectServo aDirectServoMotion = new DirectServo(_lbr.getCurrentJointPosition());
        
        aDirectServoMotion.setJointVelocityRel(0.1);
        aDirectServoMotion.setMinimumTrajectoryExecutionTime(40e-3);

        _toolAttachedToLBR.moveAsync(aDirectServoMotion);
        IDirectServoRuntime theDirectServoRuntime = aDirectServoMotion.getRuntime();

        Frame destFrame = theDirectServoRuntime.getCurrentCartesianDestination(_toolAttachedToLBR.getDefaultMotionFrame());
        
        
        try
        {  
        	while(KUKAServerManager.directServoMotionFlag_ == false)
        		ThreadUtil.milliSleep(100);
        	
	        while(KUKAServerManager.directServoMotionFlag_ == true){
	        	
	        	int cnt=0;
	        	while(KUKAServerManager.jRelVelServo_!=0 && (KUKAServerManager.EEFPos_Servo_[0]>200 ||
	        		KUKAServerManager.EEFPos_Servo_[1]>200 || KUKAServerManager.EEFPos_Servo_[2]>200)){
	        		  
		            while(cartesianDistance(destFrame,KUKAServerManager.EEFPos_Servo_) > 0.5 || 
		            		angleDistance(destFrame,KUKAServerManager.EEFPos_Servo_) > 0.5*Math.PI/180)
		            {            	
		                theDirectServoRuntime.updateWithRealtimeSystem();
		                aDirectServoMotion.setJointVelocityRel(KUKAServerManager.jRelVelServo_);
		               
		                // Get the measured position in Cartesian...
		                Frame msrPose = theDirectServoRuntime
		                        .getCurrentCartesianDestination(_toolAttachedToLBR.getDefaultMotionFrame());
		
		                ThreadUtil.milliSleep(MILLI_SLEEP_TO_EMULATE_COMPUTATIONAL_EFFORT);
	                
	                	if((cnt + 1) % 500 == 0){
	                		logger_.info("the No. of the control point:" + (cnt+1)*500);
	                		logger_.info("Servo Velocity" + Double.toString(KUKAServerManager.jRelVelServo_));
	                	}
	                	
	                	//TODO:180度的时候仍需考虑，KUKA中180和-180几乎没有区别，有时可能会卡住
	                	// compute a new commanded position
		                destFrame.setX(destFrame.getX() - (destFrame.getX() - KUKAServerManager.EEFPos_Servo_[0])/100);
		                destFrame.setY(destFrame.getY() - (destFrame.getY() - KUKAServerManager.EEFPos_Servo_[1])/100);
		                destFrame.setZ(destFrame.getZ() - (destFrame.getZ() - KUKAServerManager.EEFPos_Servo_[2])/100);                             
		                destFrame.setAlphaRad(destFrame.getAlphaRad() - (destFrame.getAlphaRad() - KUKAServerManager.EEFPos_Servo_[3])/100);
	                	destFrame.setBetaRad(destFrame.getBetaRad() - (destFrame.getBetaRad() - KUKAServerManager.EEFPos_Servo_[4])/100);
	                	destFrame.setGammaRad(destFrame.getGammaRad() - (destFrame.getGammaRad() - KUKAServerManager.EEFPos_Servo_[5])/100);
	                
	                	if (doDebugPrints){
	                		logger_.info("New cartesian goal " + destFrame);
	                		logger_.info("LBR position " + _lbr.getCurrentCartesianPosition(_lbr.getFlange()));
	                		logger_.info("Measured cartesian pose from runtime： " + msrPose);
	                	}
	                	   
	                	theDirectServoRuntime.setDestination(destFrame);
	                	++cnt;
	          	  }
	        	}
        	}
        
        } catch (Exception e) {
            logger_.info(e.getLocalizedMessage());
            e.printStackTrace();
        }

        logger_.info("Stop the DirectServo motion");
        theDirectServoRuntime.stopMotion();
		
	}
}