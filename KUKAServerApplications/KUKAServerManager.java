package KUKAServerApplications;

import java.util.concurrent.atomic.AtomicBoolean;

import com.kuka.med.deviceModel.LBRMed;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.geometricModel.LoadData;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.math.XyzAbcTransformation;

public class KUKAServerManager extends RoboticsAPIApplication
{
	//@Inject
	public static LBRMed LBR_Med_;

	//@Inject
	private Controller kuka_Sunrise_Cabinet_;

	// Tool Data
	public static Tool tool_;
    private LoadData _loadData;

    private static final String TOOL_FRAME = "toolFrame";
    private static final double[] TRANSLATION_OF_TOOL = { 0, 0, 0 };
    private static final double MASS = 0;
    private static final double[] CENTER_OF_MASS_IN_MILLIMETER = { 0, 0, 0 };

	// Utility classes
	public static MediaFlangeFunctions mff_; // utility functions to read Write
	public static StateQueryMachine queryRobotStateVariables_;

	//public static StateVariablesOfRobot queryStateVariables; // state variables publisher
	private BackgroundTask dabak_; // server function.
	private QueryStateTaskServer queryStateServer_;

	public static int jNum_;

	public static double jDispMax_[];
	public static double updateCycleJointPos_[];
	
	// PTP variables 相对速度设置为0.1-0.15
	public static double jRelVel_;
	public static double cVel_;
	public static double jRelVelServo_;

	public static double[] jPos_;
	public static double[] EEFPos_;
	public static double[] EEFOffset_;
	public static double[] EEFPosCIRC1_;
	public static double[] EEFPosCIRC2_;
	public static double[] jVelArray_;

	public static double[] EEFPos_Servo_;
	public static double[] jVelOld_;
	
	// 原子Boolean，线程安全地设置_handGuiding_endFlag的布尔值
	public static AtomicBoolean handguiding_endflag_ = new AtomicBoolean(false);
	
	public static boolean terminate_flag_ = false;
	public static boolean directServoMotionFlag_ = false;

	public static boolean background_thread_ = false;
	public static boolean send_thread_ = false;

	@Override
	public void initialize()
	{
		// TODO 使用@Inject后，不需要再实例化对象
		LBR_Med_ = this.getContext().getDeviceFromType(LBRMed.class);
		kuka_Sunrise_Cabinet_ = getController("KUKA_Sunrise_Cabinet_1");
		
        _loadData = new LoadData();
        _loadData.setMass(MASS);
        _loadData.setCenterOfMass(
                CENTER_OF_MASS_IN_MILLIMETER[0], CENTER_OF_MASS_IN_MILLIMETER[1],
                CENTER_OF_MASS_IN_MILLIMETER[2]);
        
        tool_ = new Tool("Tool",_loadData);
        
        XyzAbcTransformation trans = XyzAbcTransformation.ofTranslation(
                TRANSLATION_OF_TOOL[0], TRANSLATION_OF_TOOL[1],
                TRANSLATION_OF_TOOL[2]);
        
        ObjectFrame aTransformation = tool_.addChildFrame(TOOL_FRAME
                + "(TCP)", trans);
        
        tool_.setDefaultMotionFrame(aTransformation);
        // Attach tool to the robot
        tool_.attachTo(LBR_Med_.getFlange());

		jNum_ = LBR_Med_.getJointCount();

		// Initialize the variables
		cVel_ = 0;
		jRelVel_ = 0;
		jRelVelServo_ = 0;

		jPos_ = new double[]{0., 0., 0., 0., 0., 0., 0.};
		jDispMax_ = new double[]{0., 0., 0., 0., 0., 0., 0.};
		jVelArray_ = new double[]{0., 0., 0., 0., 0., 0., 0.};
		jVelOld_ = new double[]{0., 0., 0., 0., 0., 0., 0.};

		EEFPos_ = new double[]{0., 0., 0., 0., 0., 0., 0.};
		EEFOffset_ = new double[]{0., 0., 0.};
		EEFPosCIRC1_ = new double[]{0., 0., 0., 0., 0., 0., 0.};
		EEFPosCIRC2_ = new double[]{0., 0., 0., 0., 0., 0., 0.};
		EEFPos_Servo_ = new double[]{0., 0., 0., 0., 0., 0., 0.};
		updateCycleJointPos_ = new double[]{0., 0., 0., 0., 0., 0., 0.};

		terminate_flag_ = false;

		// 开启“查询状态服务器”子线程
		int _sendStatePort = 30003;
		int _timeout = 300 * 1000; 
		queryStateServer_ = new QueryStateTaskServer(_sendStatePort, _timeout);// 设置300秒的等待时间
		queryStateServer_.start();

		// 开启“服务器”主线程
		int _backgroundPort = 30001;
		dabak_ = new BackgroundTask(_backgroundPort, _timeout, LBR_Med_, this.getLogger());
		dabak_.start();
		
		// 设置工具坐标系为法兰坐标系
		double[] defaultToolInfo = { 0., 0., 0., 0., 0., 0. ,0., 0., 0., 0.};
		dabak_.attachToolToFlange("defaultToolEqualFlange", defaultToolInfo);
		
		// Call instructors of utility classes
		mff_ = new MediaFlangeFunctions(kuka_Sunrise_Cabinet_, LBR_Med_, dabak_);
		queryRobotStateVariables_ = new StateQueryMachine(LBR_Med_);
	}

	@Override
	public void run()
	{

		// TODO 判断run运行结束时对子线程的影响，如何关闭socket
		do {
			try {
				Thread.currentThread();
				// sleep 100ms
				Thread.sleep(100);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		} while (background_thread_ == false);

		do {
			try {
				Thread.currentThread();
				// sleep 100ms
				Thread.sleep(100);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		} while (send_thread_ == false);


		try {
			// TODO terminateFlag的终止条件选择设置
			while (terminate_flag_ == false){
				///////////////////////////////////////////
				///////////////////////////////////////////
			}
		} catch (Exception e) {
			getLogger().error("error!!!");
			getLogger().error(e.toString());
		}

		try {
			Thread.sleep(1000);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

	// Main routine, which starts the application
	public static void main(String[] args) {
		KUKAServerManager app = new KUKAServerManager();
		app.runApplication();
	}

}
