package servoTest;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.lin;
import com.kuka.common.StatisticTimer;
import com.kuka.common.ThreadUtil;
import com.kuka.common.StatisticTimer.OneTimeStep;
import com.kuka.connectivity.motionModel.smartServoLIN.ISmartServoLINRuntime;
import com.kuka.connectivity.motionModel.smartServoLIN.SmartServoLIN;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.AbstractFrame;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.LoadData;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.math.XyzAbcTransformation;

public class ThreePointTestSmartServoLIN extends RoboticsAPIApplication
{
	    private LBR _lbr;
	    private Tool _toolAttachedToLBR;
	    private LoadData _loadData;
	    private ISmartServoLINRuntime _smartServoLINRuntime = null;

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

	        AbstractFrame initialPosition = _lbr.getCurrentCartesianPosition(_lbr
	                .getFlange());

	        // Create a new smart servo linear motion
	        SmartServoLIN aSmartServoLINMotion = new SmartServoLIN(initialPosition);

	        aSmartServoLINMotion.setMinimumTrajectoryExecutionTime(20e-3);
	        double maxTranslationVelocity[] = {40, 40, 40};
	        double maxOrientationVelocity[] = {0.05, 0.05, 0.05};
	        aSmartServoLINMotion.setMaxTranslationVelocity(maxTranslationVelocity);
	        aSmartServoLINMotion.setMaxOrientationVelocity(maxOrientationVelocity);

	        getLogger().info("Starting the SmartServoLIN in position control mode");
	        _lbr.getFlange().moveAsync(aSmartServoLINMotion);

	        getLogger().info("Get the runtime of the SmartServoLIN motion");
	        _smartServoLINRuntime = aSmartServoLINMotion.getRuntime();

	        StatisticTimer timing = new StatisticTimer();

	        // Start the smart servo lin sine movement
	        timing = startThreePointMovement(_smartServoLINRuntime, timing);

	        ThreadUtil.milliSleep(1000);
	        getLogger().info("Print statistic timing");
	        getLogger().info(getClass().getName() + _smartServoLINRuntime.toString());

	        getLogger().info("Stop the SmartServoLIN motion");
	        _smartServoLINRuntime.stopMotion();
	        
	    }

	    private StatisticTimer startThreePointMovement(
	            ISmartServoLINRuntime smartServoLINRuntime, StatisticTimer timing)
	    {
	        Frame aFrame = smartServoLINRuntime.getCurrentCartesianDestination(_lbr.getFlange());

	        try
	        {
	                final OneTimeStep aStep = timing.newTimeStep();

	                Frame cFrame = new Frame(aFrame);

	        		cFrame.setZ(200);
	        		getLogger().info("move to the auxiliary point");
	                smartServoLINRuntime.setDestination(cFrame);
	                getLogger().info("reach the auxiliary point.");
	                
	                ThreadUtil.milliSleep(2500);      

	        		cFrame.setY(200);
	        		getLogger().info("move to the end point.");
	        		smartServoLINRuntime.setDestination(cFrame);
	                getLogger().info("reach the end point.");
	        		
	                ThreadUtil.milliSleep(10000);
	                
	                aStep.end();
	        }
	        catch (Exception e)
	        {
	            getLogger().error(e.getLocalizedMessage());
	            e.printStackTrace();
	        }
	        return timing;
	    }
	    
public static void main(String args[]){
	ThreePointTestSmartServoLIN sample = new ThreePointTestSmartServoLIN();
	sample.runApplication();
}
}
