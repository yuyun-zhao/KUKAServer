package servoTest;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;
import com.kuka.common.ThreadUtil;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.LoadData;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.math.XyzAbcTransformation;

public class PTPtest extends RoboticsAPIApplication
{
	private LBR _lbr;
	private Tool _toolAttachedToLBR;
	private LoadData _loadData;
	
	// Tool Data
	private static final String TOOL_FRAME = "toolFrame";
	private static final double[] TRANSLATION_OF_TOOL = { 0, 0, 0 };
	private static final double MASS = 0;
	private static final double[] CENTER_OF_MASS_IN_MILLIMETER = { 0, 0, 0 };
	
	public void initialize(){
		
		_lbr =getContext().getDeviceFromType(LBR.class);
		
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
	
	public void moveToInitialPosition(){
		
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
		
	}
	
	public void moveToEndPosition()
    {
		//Frame bFrame = _lbr.getCurrentCartesianPosition(_toolAttachedToLBR.getDefaultMotionFrame());
		
		//bFrame.setX(-50);
		//bFrame.setY(560);
		//bFrame.setZ(700);
		//bFrame.setAlphaRad(-1.4);
		//bFrame.setBetaRad(0.2);
		//bFrame.setGammaRad(-3);
		
		//_toolAttachedToLBR.move(ptp(bFrame).setJointVelocityRel(0.1));
		
        _toolAttachedToLBR.move(
               ptp(Math.PI / 180 * 90., Math.PI / 180 * 20., Math.PI / 180 * 10., -Math.PI / 180 * 60., 
               		-Math.PI / 180 * 10., Math.PI / 180 * 90., 0.).setJointVelocityRel(0.1));
    }
	
	@Override
	public void run() throws Exception {
		
		try{
			
			moveToInitialPosition();
			getLogger().info("move to the initial position.");
			Frame msrPos = _lbr.getCurrentCartesianPosition(_toolAttachedToLBR.getDefaultMotionFrame());
			getLogger().info("Measured Cartesian Position:" + msrPos);
			
			ThreadUtil.milliSleep(5000);
		
			moveToEndPosition();
			getLogger().info("move to the end postion.");
			msrPos = _lbr.getCurrentCartesianPosition(_toolAttachedToLBR.getDefaultMotionFrame());
			getLogger().info("Measured Cartesian Position:" + msrPos);
			
		} catch(Exception e){
		    getLogger().info(e.getLocalizedMessage());
	        e.printStackTrace();
		}
	}
	
    /**
     * Main routine, which starts the application
     */
    public static void main(String[] args)
    {
        PTPtest sample = new PTPtest();
        sample.runApplication();
    }
}

