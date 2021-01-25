package servoTest;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.lin;
import com.kuka.common.ThreadUtil;
import com.kuka.connectivity.motionModel.smartServo.ISmartServoRuntime;
import com.kuka.connectivity.motionModel.smartServo.SmartServo;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.LoadData;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.math.XyzAbcTransformation;

public class ThreePointTestSmartServo extends RoboticsAPIApplication
{
	    private LBR _lbr;
	    private Tool _toolAttachedToLBR;
	    private LoadData _loadData;

	    // Tool Data
	    private static final String TOOL_FRAME = "toolFrame";
	    private static final double[] TRANSLATION_OF_TOOL = { 0, 0, 0 };
	    private static final double MASS = 0;
	    private static final double[] CENTER_OF_MASS_IN_MILLIMETER = { 0, 0, 0 };

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

		public void moveToInitialPosition()
		{	
			Frame aFrame = _lbr.getCurrentCartesianPosition(_toolAttachedToLBR.getDefaultMotionFrame());
			aFrame.setX(610);
			aFrame.setY(0);
			aFrame.setZ(600);
			aFrame.setAlphaRad(Math.PI);
			aFrame.setBetaRad(0);
			aFrame.setGammaRad(Math.PI);
			
			_lbr.move(lin(aFrame).setJointVelocityRel(0.1));
		}
	    
	    @Override
	    public void run()
	    {
	        getLogger().info("Move to start position.");
	        moveToInitialPosition();
	        ThreadUtil.milliSleep(3000);

	        SmartServo aSmartServoMotion = new SmartServo(_lbr.getCurrentJointPosition());

	        aSmartServoMotion.useTrace(true);

	        aSmartServoMotion.setJointVelocityRel(0.1);
	        aSmartServoMotion.setMinimumTrajectoryExecutionTime(5e-3);

	        getLogger().info("Starting SmartServo motion in position control mode");
	        _toolAttachedToLBR.moveAsync(aSmartServoMotion);

	        getLogger().info("Get the runtime of the SmartServo motion");
	        ISmartServoRuntime theServoRuntime = aSmartServoMotion.getRuntime();

	        Frame aFrame = theServoRuntime.getCurrentCartesianDestination(_toolAttachedToLBR.getDefaultMotionFrame());

	        try
	        {
	                Frame cFrame = new Frame(aFrame);

	        		cFrame.setZ(200);
	        		getLogger().info("move to the auxiliary point");
	                theServoRuntime.setDestination(cFrame);
	                getLogger().info("reach the auxiliary point.");
	                
	                ThreadUtil.milliSleep(1500);      

	        		cFrame.setY(200);
	        		getLogger().info("move to the end point.");
	        		theServoRuntime.setDestination(cFrame);
	                getLogger().info("reach the end point.");
	        		
	                ThreadUtil.milliSleep(10000);
	               
	        } catch (Exception e) {
	            getLogger().error(e.getLocalizedMessage());
	            e.printStackTrace();
	        }
	    }
	    
public static void main(String args[]){
	ThreePointTestSmartServo sample = new ThreePointTestSmartServo();
	sample.runApplication();
}
}
