package servoTest;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;
import com.kuka.common.ThreadUtil;

import com.kuka.connectivity.motionModel.directServo.DirectServo;
import com.kuka.connectivity.motionModel.directServo.IDirectServoRuntime;
import com.kuka.connectivity.motionModel.smartServo.ServoMotion;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.LoadData;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.math.XyzAbcTransformation;

public class testDirectServo extends RoboticsAPIApplication
{
	private LBR _lbr;
    private Tool _toolAttachedToLBR;
    private LoadData _loadData;

    // Tool Data

    private static final String TOOL_FRAME = "toolFrame";
    private static final double[] TRANSLATION_OF_TOOL = { 0, 0, 0 };
    private static final double MASS = 0;
    private static final double[] CENTER_OF_MASS_IN_MILLIMETER = { 0, 0, 0 };
    
//    private static final double STEP_LENGTH = 1;
    private static final int MILLI_SLEEP_TO_EMULATE_COMPUTATIONAL_EFFORT = 30;
    
    @Override
    public void initialize()
    {
        _lbr = getContext().getDeviceFromType(LBR.class);

        // Create a Tool by Hand this is the tool we want to move with some mass
        // properties and a TCP-Z-offset of 100.
        _loadData = new LoadData();
        _loadData.setMass(MASS);
        _loadData.setCenterOfMass(
                CENTER_OF_MASS_IN_MILLIMETER[0], CENTER_OF_MASS_IN_MILLIMETER[1],
                CENTER_OF_MASS_IN_MILLIMETER[2]);
        
        _toolAttachedToLBR = new Tool("Tool", _loadData);

        XyzAbcTransformation trans = XyzAbcTransformation.ofTranslation(
                TRANSLATION_OF_TOOL[0], TRANSLATION_OF_TOOL[1],
                TRANSLATION_OF_TOOL[2]);
        
        ObjectFrame aTransformation = _toolAttachedToLBR.addChildFrame(TOOL_FRAME
                + "(TCP)", trans);
        
        _toolAttachedToLBR.setDefaultMotionFrame(aTransformation);
        // Attach tool to the robot
        _toolAttachedToLBR.attachTo(_lbr.getFlange());
        
    }
    
    /**
     * Move to an initial Position WARNING: MAKE SURE, THAT the pose is collision free.
     */
    private void moveToInitialPosition()
    {	
		Frame aFrame = _lbr.getCurrentCartesianPosition(_toolAttachedToLBR.getDefaultMotionFrame());
		aFrame.setX(610);
		aFrame.setY(0);
		aFrame.setZ(600);
		aFrame.setAlphaRad(Math.PI);
		aFrame.setBetaRad(0);
		aFrame.setGammaRad(Math.PI);
		
		_toolAttachedToLBR.move(ptp(aFrame).setJointVelocityRel(0.1));
		
		//_toolAttachedToLBR.move(ptp(0., Math.PI / 180 * 30., 0., -Math.PI / 180 * 60., 0.,
        //                Math.PI / 180 * 90., 0.).setJointVelocityRel(0.1));
		
        /* Note: The Validation itself justifies, that in this very time instance, the load parameter setting was
         * sufficient. This does not mean by far, that the parameter setting is valid in the sequel or lifetime of this
         * program */
        try
        {
            if (!ServoMotion.validateForImpedanceMode(_toolAttachedToLBR))
            {
                getLogger().info("Validation of torque model failed - correct your mass property settings");
                getLogger().info("SmartServo will be available for position controlled mode only, " +
                														   "until validation is performed");
            }
        }
        catch (IllegalStateException e)
        {
            getLogger().info("Omitting validation failure for this sample\n"
                    + e.getMessage());
        }
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
    
    /**
     * Main Application Routine
     */
    @Override
    public void run()
    {
        moveToInitialPosition();
        getLogger().info("move to initial position.");
        ThreadUtil.milliSleep(5000);
      
        //init[] = {610, 0, 600, Math.PI, 0, Math.PI};
        double end[] = {610, 200, 400, Math.PI, 0.5, Math.PI};
        
        boolean doDebugPrints = false;

        DirectServo aDirectServoMotion = new DirectServo(_lbr.getCurrentJointPosition());

        aDirectServoMotion.setJointVelocityRel(0.4);
        aDirectServoMotion.setMinimumTrajectoryExecutionTime(40e-3);

        getLogger().info("Starting DirectServo motion in position control mode");
        _toolAttachedToLBR.moveAsync(aDirectServoMotion);
        getLogger().info("Get the runtime of the DirectServo motion");
        IDirectServoRuntime theDirectServoRuntime = aDirectServoMotion.getRuntime();

        Frame destFrame = theDirectServoRuntime.getCurrentCartesianDestination(_toolAttachedToLBR.getDefaultMotionFrame());
        int cnt=0;
        
        try
        {
            while(cartesianDistance(destFrame,end) > 0.2 || angleDistance(destFrame,end) > 0.2*Math.PI/180)
            {            	
                theDirectServoRuntime.updateWithRealtimeSystem();
                
                // Get the measured position in Cartesian...
                Frame msrPose = theDirectServoRuntime
                        .getCurrentCartesianDestination(_toolAttachedToLBR.getDefaultMotionFrame());

                ThreadUtil.milliSleep(MILLI_SLEEP_TO_EMULATE_COMPUTATIONAL_EFFORT);
                
                if((cnt + 1) % 10 == 0)getLogger().info("the No. of the control point:" + (cnt+1)*10);

                // compute a new commanded position
                destFrame.setX(destFrame.getX() - (destFrame.getX() - end[0])/1000);
                destFrame.setY(destFrame.getY() - (destFrame.getY() - end[1])/1000);
                destFrame.setZ(destFrame.getZ() - (destFrame.getZ() - end[2])/1000);                             
                destFrame.setAlphaRad(destFrame.getAlphaRad() - (destFrame.getAlphaRad() - end[3])/1000);
                destFrame.setBetaRad(destFrame.getBetaRad() - (destFrame.getBetaRad() - end[4])/1000);
                destFrame.setGammaRad(destFrame.getGammaRad() - (destFrame.getGammaRad() - end[5])/1000);
                
                if (doDebugPrints){
                    getLogger().info("New cartesian goal " + destFrame);
                    getLogger().info("LBR position " + _lbr.getCurrentCartesianPosition(_lbr.getFlange()));
                    getLogger().info("Measured cartesian pose from runtime " + msrPose);
                }
              
                theDirectServoRuntime.setDestination(destFrame);
                ++cnt;
            }
        } catch (Exception e) {
            getLogger().info(e.getLocalizedMessage());
            e.printStackTrace();
        }

        //Print statistics and parameters of the motion
        getLogger().info("Simple Cartesian Test \n" + theDirectServoRuntime.toString());

        getLogger().info("Stop the DirectServo motion");
        theDirectServoRuntime.stopMotion();
    }


    /**
     * Main routine, which starts the application
     */
    public static void main(String[] args)
    {
        testDirectServo app = new testDirectServo();
        app.runApplication();
    }
}
