package KUKAServerApplications;

/* By Mohammad SAFEEA: Coimbra University-Portugal, 
 * Ensam University-France
 * 
 * KST 1.7
 *  
 * First upload 07-May-2017
 * 
 * Final update 26th-06-2018 
 * 
 * This is a multi-threaded server program that is meant to be used with both
 *    KUKA iiwa 7 R 800
 * or KUKA iiwa 14 R 820.
 * The robot shall be provided with a pneumatic flange, 
 * the server of this application listens on the port 30001.
 * 
 * */

import javax.inject.Inject;

import com.kuka.connectivity.motionModel.directServo.DirectServo;
import com.kuka.connectivity.motionModel.directServo.IDirectServoRuntime;
//import com.kuka.generated.ioAccess.MediaFlangeIOGroup;
import com.kuka.med.deviceModel.LBRMed;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.geometricModel.Frame;
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

	public static double[] jPos_;
	public static double[] EEFPos_;
	public static double[] EEFOffset_;
	public static double[] EEFPosCIRC1_;
	public static double[] EEFPosCIRC2_;
	public static double[] jVelArray_;

	public static double[] EEFPos_Servo_;
	public static double[] jVelOld_;

	public static boolean terminate_flag_ = false;
	public static boolean directSmart_ServoMotionFlag_ = false;

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
			while (terminate_flag_ == false)
			{

//				// 尚未用到，与servo有关
//				if (CommandParseMachine.isPrefix(_command, "stDcEEf_"))
//				{
//					directSmart_ServoMotionFlag = true;
//					dabak.sendCommand(ack);
//					_command.delete(0, _command.length());
//					getLogger().info(
//							"realtime control in Cartesian space initiated");
//					directServoStartCartesian();
//					getLogger().info("realtime control terminated");
//
//				}
//				// realtime velocity control in joint space
//				else if (CommandParseMachine.isPrefix(_command, "stVelDcJoint_"))
//				{
//					directSmart_ServoMotionFlag = true;
//					dabak.sendCommand(ack);
//					_command.delete(0, _command.length());
//					getLogger()
//							.info("Velocity-mode, realtime control in joint space initiated");
//					directServoStartVelMode();
//					getLogger().info("realtime control terminated");
//
//				}
//				else if (CommandParseMachine.isPrefix(_command, "startSmartImpedneceJoints"))
//				{
//					// pre-processing step
//					directSmart_ServoMotionFlag = true;
//					double[] variables = SmartServoWithImpedence
//							.getControlParameters(_command.toString());
//					if (variables.length > 6) {
//						double massOfTool = variables[0];
//						double[] toolsCOMCoordiantes = { variables[1],
//								variables[2], variables[3] };
//						double cStifness = variables[4];
//						double rStifness = variables[5];
//						double nStifness = variables[6];
//						getLogger()
//								.info("Mass: " + Double.toString(massOfTool));
//						getLogger()
//								.info("COM X : "
//										+ Double.toString(toolsCOMCoordiantes[0]));
//						getLogger()
//								.info("COM Y : "
//										+ Double.toString(toolsCOMCoordiantes[1]));
//						getLogger()
//								.info("COM Z : "
//										+ Double.toString(toolsCOMCoordiantes[2]));
//						getLogger().info(
//								"Stiffness C: " + Double.toString(cStifness));
//						getLogger().info(
//								"Stiffness R: " + Double.toString(rStifness));
//						getLogger().info(
//								"Stiffness N: " + Double.toString(nStifness));
//						dabak.sendCommand(ack);
//						_command.delete(0, _command.length());
//
//						try {
//							getLogger().info("realtime control initiated");
//							SmartServoWithImpedence.startRealTimeWithImpedence(
//									_lbr_Med, kuka_Sunrise_Cabinet, massOfTool,
//									toolsCOMCoordiantes, cStifness, rStifness,
//									nStifness);
//						} catch (Exception e) {
//							// TODO: handle exception
//							getLogger().error(e.toString());
//						}
//						getLogger().info("realtime control terminated");
//						dabak.sendCommand(ack);
//						_command.delete(0, _command.length());
//					}
//
//				}
//
//
//				// PTP motion with condition instructions
//				else if (CommandParseMachine.isPrefix(_command, "doPTP!"))
//				{
//					if (CommandParseMachine.isPrefix(
//							_command, "doPTP!inJS")) {
//						double[] indices = new double[7];
//						double[] maxTorque = new double[7];
//						double[] minTorque = new double[7];
//						int n = CommandParseMachine
//								.get_Indexes_ValBoundaries(
//										_command.toString(), indices,
//										minTorque, maxTorque);
//						dabak.sendCommand(ack);
//						_command.delete(0, _command.length());
//						for (int i = 0; i < n; i++) {
//							String strInfo = "[minTorque,maxTorque] for joint "
//									+ Double.toString(indices[i]) + " is "
//									+ Double.toString(minTorque[i]) + " , "
//									+ Double.toString(maxTorque[i]);
//							getLogger().info(strInfo);
//						}
//						MotionExecuteMachine.PTPMotionJointSpaceTorquesConditional(n,
//								indices, minTorque, maxTorque);
//					} else if (CommandParseMachine
//							.isPrefix(_command, "doPTP!inCS")) {
//						double[] indices = new double[7];
//						double[] maxTorque = new double[7];
//						double[] minTorque = new double[7];
//						int n = CommandParseMachine
//								.get_Indexes_ValBoundaries(
//										_command.toString(), indices,
//										minTorque, maxTorque);
//						dabak.sendCommand(ack);
//						_command.delete(0, _command.length());
//						for (int i = 0; i < n; i++) {
//							String strInfo = "[minTorque,maxTorque] for joint "
//									+ Double.toString(indices[i]) + " is "
//									+ Double.toString(minTorque[i]) + " , "
//									+ Double.toString(maxTorque[i]);
//							getLogger().info(strInfo);
//						}
//						MotionExecuteMachine
//								.PTPMotionLineCartesianSpaceTorquesConditional(
//										n, indices, minTorque, maxTorque);
//					} else if (CommandParseMachine
//							.isPrefix(_command,
//									"doPTP!CSCircle1")) {
//						double[] indices = new double[7];
//						double[] maxTorque = new double[7];
//						double[] minTorque = new double[7];
//						int n = CommandParseMachine
//								.get_Indexes_ValBoundaries(
//										_command.toString(), indices,
//										minTorque, maxTorque);
//						dabak.sendCommand(ack);
//						_command.delete(0, _command.length());
//						for (int i = 0; i < n; i++) {
//							String strInfo = "[minTorque,maxTorque] for joint "
//									+ Double.toString(indices[i]) + " is "
//									+ Double.toString(minTorque[i]) + " , "
//									+ Double.toString(maxTorque[i]);
//							getLogger().info(strInfo);
//						}
//						MotionExecuteMachine.PTPMotionJointSpaceTorquesConditional(n,
//								indices, minTorque, maxTorque);
//					}
//
//				}
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

	// TODO 是否放到back
//	public void attachToolToFlange(String TOOL_FRAME, double[] TRANSFORM_OF_TOOL)
//	{
//		double[] CENTER_OF_MASS_IN_MILLIMETER = { 0., 0., 0. };
//		double MASS = 0.;
//
//		// 负载数据，设置为0代表工具质量为0，即没有外加工具
//		LoadData loadData = new LoadData();
//		loadData.setMass(MASS);
//		loadData.setCenterOfMass(CENTER_OF_MASS_IN_MILLIMETER[0],
//				CENTER_OF_MASS_IN_MILLIMETER[1],
//				CENTER_OF_MASS_IN_MILLIMETER[2]);
//
//		_tool = new Tool(TOOL_FRAME, loadData);
//		XyzAbcTransformation trans = XyzAbcTransformation.ofRad(
//				TRANSFORM_OF_TOOL[0], TRANSFORM_OF_TOOL[1],
//				TRANSFORM_OF_TOOL[2], TRANSFORM_OF_TOOL[3],
//				TRANSFORM_OF_TOOL[4], TRANSFORM_OF_TOOL[5]);
//
//		// 为tool添加坐标系
//		ObjectFrame aTransformation = _tool.addChildFrame(
//				TOOL_FRAME + "(TCP)", trans);
//		_tool.setDefaultMotionFrame(aTransformation);
//
//		// Attach tool to the robot
//		_tool.attachTo(_lbr_Med.getFlange());
//	}

	// /////////////////////////////////
	// Velocity-mode, joint space
	public void directServoStartVelMode() {

		boolean doDebugPrints = false;

		JointPosition initialPosition = new JointPosition(
				LBR_Med_.getCurrentJointPosition());
		// Initiate joints velocities and positions
		double[] jPOS = new double[7];
		for (int i = 0; i < 7; i++) {
			jVelArray_[i] = 0;
			jVelOld_[i] = 0;
			jPOS[i] = initialPosition.get(i);
		}

		DirectServo aDirectServoMotion = new DirectServo(initialPosition);

		aDirectServoMotion.setMinimumTrajectoryExecutionTime(40e-3);

		getLogger().info("Starting realtime velocity control mode");
		LBR_Med_.moveAsync(aDirectServoMotion);

		getLogger().info("Get the runtime of the DirectServo motion");
		IDirectServoRuntime theDirectServoRuntime = aDirectServoMotion
				.getRuntime();

		JointPosition destination = new JointPosition(LBR_Med_.getJointCount());

		try {
			// do a cyclic loop
			// Do some timing...
			// in nanosec
			double dt = 0;
			long t0 = System.nanoTime();
			long t1 = t0;
			while (directSmart_ServoMotionFlag_ == true) {

				// ///////////////////////////////////////////////////////
				// Insert your code here
				// e.g Visual Servoing or the like
				// Synchronize with the realtime system
				// theDirectServoRuntime.updateWithRealtimeSystem();

				if (doDebugPrints) {
					getLogger().info("Current fifth joint position " + jPos_[5]);
					getLogger().info("Current joint destination "
							+ theDirectServoRuntime.getCurrentJointDestination());
				}

				Thread.sleep(1);
				// getLogger().warn(Double.toString(jpos[0]));
				// getLogger().info(daCommand);
				JointPosition currentPos = new JointPosition(
						LBR_Med_.getCurrentJointPosition());

				t1 = System.nanoTime();
				dt = ((double) (t1 - t0)) / (1e9);
				t0 = t1;
				updatejointsPosFromVelAcc(jPOS, dt);
				for (int k = 0; k < destination.getAxisCount(); ++k) {
					destination.set(k, jPOS[k]);
				}
				theDirectServoRuntime.setDestination(destination);
			}
		} catch (Exception e) {
			getLogger().info(e.getLocalizedMessage());
			e.printStackTrace();
			// Print statistics and parameters of the motion
			getLogger().info(
					"Simple Cartesian Test \n"
							+ theDirectServoRuntime.toString());
			getLogger().info("Stop the realtime velocity control mode");

		}

		theDirectServoRuntime.stopMotion();
		getLogger().info("Stop realtime velocity control mode");
	}

	// updates joints positions by integration, Midpoint Rieman Sum is utilized
	private void updatejointsPosFromVelAcc(double[] jPOS, double dt) {
		for (int i = 0; i < 7; i++) {
			jPOS[i] += (jVelArray_[i] + jVelOld_[i]) * dt / 2;
			// update old joints velocities
			jVelOld_[i] = jVelArray_[i];
		}
	}


	private void directServoStartCartesian() {

		boolean doDebugPrints = false;

		DirectServo aDirectServoMotion = new DirectServo(
				LBR_Med_.getCurrentJointPosition());

		aDirectServoMotion.setMinimumTrajectoryExecutionTime(40e-3);

		getLogger()
				.info("Starting DirectServo motion in position control mode");
		KUKAServerManager.tool_.moveAsync(aDirectServoMotion);

		getLogger().info("Get the runtime of the DirectServo motion");
		IDirectServoRuntime theDirectServoRuntime = aDirectServoMotion
				.getRuntime();

		Frame aFrame = theDirectServoRuntime
				.getCurrentCartesianDestination(KUKAServerManager.tool_
						.getDefaultMotionFrame());
		Frame destFrame = aFrame.copyWithRedundancy();
		// Initiate the initial position
		EEFPos_Servo_[0] = aFrame.getX();
		EEFPos_Servo_[1] = aFrame.getY();
		EEFPos_Servo_[2] = aFrame.getZ();
		EEFPos_Servo_[3] = aFrame.getAlphaRad();
		EEFPos_Servo_[4] = aFrame.getBetaRad();
		EEFPos_Servo_[5] = aFrame.getGammaRad();

		try {
			// do a cyclic loop
			// Do some timing...
			// in nanosec

			while (directSmart_ServoMotionFlag_ == true) {

				// ///////////////////////////////////////////////////////
				// Insert your code here
				// e.g Visual Servoing or the like
				// Synchronize with the realtime system
				theDirectServoRuntime.updateWithRealtimeSystem();
				Frame msrPose = theDirectServoRuntime
						.getCurrentCartesianDestination(KUKAServerManager.tool_
								.getDefaultMotionFrame());

				if (doDebugPrints) {
					getLogger().info("Current cartesian goal " + aFrame);
					getLogger().info(
							"Current joint destination "
									+ theDirectServoRuntime
											.getCurrentJointDestination());
				}

				Thread.sleep(1);

				// update Cartesian positions
				destFrame.setX(EEFPos_Servo_[0]);
				destFrame.setY(EEFPos_Servo_[1]);
				destFrame.setZ(EEFPos_Servo_[2]);
				destFrame.setAlphaRad(EEFPos_Servo_[3]);
				destFrame.setBetaRad(EEFPos_Servo_[4]);
				destFrame.setGammaRad(EEFPos_Servo_[5]);

				if (doDebugPrints) {
					getLogger().info("New cartesian goal " + destFrame);
					getLogger().info(
							"LBR position "
									+ LBR_Med_.getCurrentCartesianPosition(LBR_Med_
											.getFlange()));
					getLogger().info(
							"Measured cartesian pose from runtime " + msrPose);
				}

				theDirectServoRuntime.setDestination(destFrame);

			}
		} catch (Exception e) {
			getLogger().info(e.getLocalizedMessage());
			e.printStackTrace();
			// Print statistics and parameters of the motion
			getLogger().info(
					"Simple Cartesian Test \n"
							+ theDirectServoRuntime.toString());

			getLogger().info("Stop the DirectServo motion");

		}
		theDirectServoRuntime.stopMotion();
		getLogger()
				.info("Stop the DirectServo motion for the stop instruction was sent");

	}



	// Main routine, which starts the application
	public static void main(String[] args) {
		KUKAServerManager app = new KUKAServerManager();
		app.runApplication();
	}

}
