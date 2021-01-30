package servoTest;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.circ;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.lin;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;

import com.kuka.common.ThreadUtil;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.LoadData;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.math.XyzAbcTransformation;

public class ThreePointTestLIN extends RoboticsAPIApplication
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
	
	public void moveToInitialPosition()
	{	
		Frame aFrame = _lbr.getCurrentCartesianPosition(_toolAttachedToLBR.getDefaultMotionFrame());
		aFrame.setX(610);
		aFrame.setY(0);
		aFrame.setZ(600);
		aFrame.setAlphaRad(Math.PI);
		aFrame.setBetaRad(0);
		aFrame.setGammaRad(Math.PI);
		
		_lbr.move(ptp(aFrame).setJointVelocityRel(0.1));
	}
	
	public void moveToAuxiliaryPosition()
    {
		Frame bFrame = _lbr.getCurrentCartesianPosition(_toolAttachedToLBR.getDefaultMotionFrame());
		
		bFrame.setX(610);
		bFrame.setY(0);
		bFrame.setZ(400);
		bFrame.setAlphaRad(Math.PI);
		bFrame.setBetaRad(0);
		bFrame.setGammaRad(Math.PI);
		
		_lbr.move(lin(bFrame).setJointVelocityRel(0.1));
    }
	
	public void moveToEndPosition()
	{
		Frame cFrame = _lbr.getCurrentCartesianPosition(_toolAttachedToLBR.getDefaultMotionFrame());
		
		cFrame.setX(610);
		cFrame.setY(200);
		cFrame.setZ(400);
		cFrame.setAlphaRad(Math.PI);
		cFrame.setBetaRad(0);
		cFrame.setGammaRad(Math.PI);
		
		_lbr.move(lin(cFrame).setJointVelocityRel(0.1));
      
		Frame bFrame = _lbr.getCurrentCartesianPosition(_toolAttachedToLBR.getDefaultMotionFrame());		
		bFrame.setX(610);bFrame.setY(0);bFrame.setZ(400);bFrame.setAlphaRad(Math.PI);
		bFrame.setBetaRad(0);bFrame.setGammaRad(Math.PI);	
		Frame aFrame = _lbr.getCurrentCartesianPosition(_toolAttachedToLBR.getDefaultMotionFrame());
		aFrame.setX(610);aFrame.setY(0);aFrame.setZ(600);aFrame.setAlphaRad(Math.PI);
		aFrame.setBetaRad(0);aFrame.setGammaRad(Math.PI);
		_lbr.move(circ(bFrame,aFrame).setJointVelocityRel(0.1));
//		//心形线		
//		ThreadUtil.milliSleep(3000);	
//		bFrame.setX(610);bFrame.setY(-100);bFrame.setZ(700);bFrame.setAlphaRad(Math.PI);
//		bFrame.setBetaRad(0);bFrame.setGammaRad(Math.PI);	
//		aFrame.setX(610);aFrame.setY(-200);aFrame.setZ(600);aFrame.setAlphaRad(Math.PI);
//		aFrame.setBetaRad(0);aFrame.setGammaRad(Math.PI);
//		_lbr.move(circ(bFrame,aFrame).setJointVelocityRel(0.1));
//		bFrame.setX(610);bFrame.setY(-50*Math.sqrt(2));bFrame.setZ(600-50*Math.sqrt(2));bFrame.setAlphaRad(Math.PI);
//		bFrame.setBetaRad(0);bFrame.setGammaRad(Math.PI);	
//		aFrame.setX(610);aFrame.setY(0);aFrame.setZ(400);aFrame.setAlphaRad(Math.PI);
//		aFrame.setBetaRad(0);aFrame.setGammaRad(Math.PI);
//		_lbr.move(circ(bFrame,aFrame).setJointVelocityRel(0.1));
//		bFrame.setX(610);bFrame.setY(50*Math.sqrt(2));bFrame.setZ(600-50*Math.sqrt(2));bFrame.setAlphaRad(Math.PI);
//		bFrame.setBetaRad(0);bFrame.setGammaRad(Math.PI);	
//		aFrame.setX(610);aFrame.setY(200);aFrame.setZ(600);aFrame.setAlphaRad(Math.PI);
//		aFrame.setBetaRad(0);aFrame.setGammaRad(Math.PI);
//		_lbr.move(circ(bFrame,aFrame).setJointVelocityRel(0.1));
//		bFrame.setX(610);bFrame.setY(100);bFrame.setZ(700);bFrame.setAlphaRad(Math.PI);
//		bFrame.setBetaRad(0);bFrame.setGammaRad(Math.PI);	
//		aFrame.setX(610);aFrame.setY(0);aFrame.setZ(600);aFrame.setAlphaRad(Math.PI);
//		aFrame.setBetaRad(0);aFrame.setGammaRad(Math.PI);
//		_lbr.move(circ(bFrame,aFrame).setJointVelocityRel(0.1));
	}
	
	@Override
	public void run() throws Exception {
		
		try{
			moveToInitialPosition();
			Frame msrPos = _lbr.getCurrentCartesianPosition(_toolAttachedToLBR.getDefaultMotionFrame());
			getLogger().info("move to the initial position.");
			getLogger().info("Measured Cartesian Position:" + msrPos);
			
			ThreadUtil.milliSleep(3000);
			
			moveToAuxiliaryPosition();
			msrPos = _lbr.getCurrentCartesianPosition(_toolAttachedToLBR.getDefaultMotionFrame());
			getLogger().info("move to the auxiliary position.");
			getLogger().info("Measured Cartesian Position:" + msrPos);
		
			moveToEndPosition();
			msrPos = _lbr.getCurrentCartesianPosition(_toolAttachedToLBR.getDefaultMotionFrame());
			getLogger().info("move to the end position.");
			getLogger().info("Measured Cartesian Position." + msrPos);			
			
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
        ThreePointTestLIN sample = new ThreePointTestLIN();
        sample.runApplication();
    }
}
