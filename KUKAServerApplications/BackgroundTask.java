package KUKAServerApplications;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.concurrent.atomic.AtomicBoolean;

import com.kuka.connectivity.motionModel.directServo.DirectServo;
import com.kuka.connectivity.motionModel.directServo.IDirectServoRuntime;
import com.kuka.med.deviceModel.LBRMed;
import com.kuka.roboticsAPI.deviceModel.JointPosition;

import com.kuka.roboticsAPI.geometricModel.LoadData;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.math.XyzAbcTransformation;
import com.kuka.task.ITaskLogger;

class BackgroundTask implements Runnable
{
	private final int port_;
	private final int timeout_;
	private final LBRMed LBR_Med_;
	private Socket socket_;
	private ServerSocket server_socket_;
	private PrintWriter pw_outputStream_;
	private BufferedReader br_inputStream_;
	public StringBuffer command_ = new StringBuffer();

	private final ITaskLogger logger_;

	// ‘\n’的ASCII码为10
	private static final String stopCharacter = Character.toString((char)(10));
	
	// 将"done\n"指令发送给上位机，表明下位机已收到指令，ack：acknowledgement
	private static final String ack = "done" + stopCharacter;
	private static final String error = "error" + stopCharacter;
	
	// 原子Boolean，线程安全地设置_handGuiding_endFlag的布尔值
	public static AtomicBoolean handguiding_endflag_ = new AtomicBoolean(false);

	BackgroundTask(int port, int timeout, LBRMed lbr_Med, ITaskLogger logger)
	{
		timeout_ = timeout;
		port_ = port;
		LBR_Med_ = lbr_Med;
		logger_ = logger;
	}

	public void sendCommand(String command_str)
	{
		pw_outputStream_.write(command_str);
		pw_outputStream_.flush();
	}

	public void start()
	{
		Thread t_background = new Thread(this);
		t_background.setDaemon(true); // 设置为守护线程
		t_background.start();
	}

	@Override
	public void run()
	{
		try {
			server_socket_ = new ServerSocket(port_);
			// 阻塞等待连接，超时后关闭socket并退出程序
			try {
				// 设置超时时间，在_timeout时间内接收不到信息就断开该socket
				server_socket_.setSoTimeout(timeout_);
				socket_ = server_socket_.accept();
			} catch (Exception e) {
				// 超时后关闭socket并直接返回
				socket_.close();
				server_socket_.close();
				
				KUKAServerManager.terminate_flag_ = true;
				return;
			}

			// 获取输入输出流
			try {
				// getOutputStream: 向客户端发消息，只能写字节数据，不能直接写字符数据
				// getInputStream: 从客户端接收数据，只能读取字节数据，不能直接读字符数据
				// PrintWriter比BufferWriter更实用，可以直接通过outputStream构造，可以直接写字符数据
				pw_outputStream_ = new PrintWriter(socket_.getOutputStream());
				// BufferWriter只能先将outputStream先转成outputStreamReader
				// BufferedReader只能先将inputStream先转成inputStreamReader，可以直接读字符数据
				br_inputStream_ = new BufferedReader(new InputStreamReader(
						socket_.getInputStream()));
			} catch (IOException e) {
				System.out.println("Error creating inPort and outPort");
				logger_.info("Error creating inPort and outPort");
			}

			KUKAServerManager.background_thread_ = true; // background thread began

			if ((socket_.isConnected()))
			{
				String msg = null;

				// TODO 设置延时关闭socket 研究何时才会关闭readLine跳出循环
				while ((msg = br_inputStream_.readLine()) != null)
				{
					command_.setLength(0);
					command_.append(msg);

					if (CommandParseMachine.isEqual(command_, "END"))
					{
						KUKAServerManager.directSmart_ServoMotionFlag_ = false;
						KUKAServerManager.terminate_flag_ = true;
						command_.setLength(0);
						break;
					}
					// 开启servo模式
					else if (CommandParseMachine.isPrefix(command_, "startDirectServoJoints")) {
						KUKAServerManager.directSmart_ServoMotionFlag_ = true;
						this.sendCommand(ack);
						command_.setLength(0);
						logger_.info("realtime control initiated");
						directServoStartJoints();
						logger_.info("realtime control terminated");
					}
					// 关闭servo模式
					else if (CommandParseMachine.isEqual(command_, "stopDirectServoJoints"))
					{
						KUKAServerManager.directSmart_ServoMotionFlag_ = false;
						this.sendCommand(ack);
						command_.setLength(0);
					}
					// 执行handGuiding
					else if (CommandParseMachine.isEqual(command_, "startHandGuiding"))
					{
						// 运行HandGuiding程序
						try{
							HandGuidingMode hg = new HandGuidingMode(LBR_Med_);
							hg.start();
							this.sendCommand(ack);
						} catch(Exception e){
							this.sendCommand(error);
							logger_.info(e.getMessage());
						}
						command_.setLength(0);
					}
					// 停止HandGuiding
					else if (CommandParseMachine.isEqual(command_, "endHandGuiding"))
					{
						handguiding_endflag_.compareAndSet(false, true);
						this.sendCommand(ack);
						command_.setLength(0);
					}
					// 响应UPDATE命令
					else if (CommandParseMachine.isPrefix(command_, "UPDATE"))
					{
						this.ResponseUPDATECommand();
					}
					// 响应QUERY命令
					else if (CommandParseMachine.isPrefix(command_, "QUERY"))
					{
						this.ResponseQUERYCommand();
					}
					// 响应MOTION指令
					else if (CommandParseMachine.isPrefix(command_, "MOTION"))
					{
						this.ResponseMOTIONCommand();
					}

					// 查询数据，目前尚未用到
					//	 inquiring data from server
					//else {
					//	this.dataAcquisitionRequest();
					//}
				}
			}
		} catch (IOException e1) {
			// 上述非try包含的过程抛出异常时，关闭socket
			try {br_inputStream_.close();}
			catch (IOException e) {e.printStackTrace();}
			
			try {pw_outputStream_.close();}
			catch (Exception e) {e.printStackTrace();}
			
			try {socket_.close();}
			catch (IOException e) {e.printStackTrace();}
			 
			try {server_socket_.close();}
			catch (IOException e) {e.printStackTrace();}
			
			KUKAServerManager.terminate_flag_ = true;
			e1.printStackTrace();
		}

		try {
			br_inputStream_.close();
			pw_outputStream_.close();
			socket_.close();
			server_socket_.close();
			KUKAServerManager.terminate_flag_ = true;
		} catch (IOException e) {
			e.printStackTrace();
		}

	}

	// 相应所有运动指令“MOTION”
	private void ResponseMOTIONCommand(){
		
		if (CommandParseMachine.isEqual(command_, "MOTION_PTPinJS")){
			
			logger_.info("MOTION_PTPinJS receive");
			try {
				MotionExecuteMachine.PTPMotionJointSpace();
				this.sendCommand(ack);
				logger_.info("MOTION_PTPinJS done");
			} catch (Exception e){
				this.sendCommand(error);
				logger_.info("MOTION_PTPinJS error");
			}
			command_.setLength(0);

		} else if (CommandParseMachine.isEqual(command_, "MOTION_PTPinCS")){
			
			logger_.info("MOTION_PTPinCS receive");
			try {
				MotionExecuteMachine.PTPMotionCartSpace();
				this.sendCommand(ack);
				logger_.info("MOTION_PTPinCS done");
			} catch (Exception e){
				this.sendCommand(error);
				logger_.info("MOTION_PTPinCS error");
				logger_.info(e.toString());
			}
			command_.setLength(0);
			
		} else if (CommandParseMachine.isEqual(command_, "MOTION_LIN")){
			
			logger_.info("MOTION_LIN receive");
			try {
				MotionExecuteMachine.LINMotion();
				this.sendCommand(ack);
				logger_.info("MOTION_LIN done");
			} catch (Exception e){
				this.sendCommand(error);
				logger_.info("MOTION_LIN error");
			}
			command_.setLength(0);

		} else if (CommandParseMachine.isEqual(command_, "MOTION_LINRelEEF")){
			 
			logger_.info("MOTION_LINRelEEF receive");
			try {
				MotionExecuteMachine.LINMotionRelEEF();
				this.sendCommand(ack);
				logger_.info("MOTION_LINRelEEF done");
			} catch (Exception e){
				this.sendCommand(error);
				logger_.info("MOTION_LINRelEEF error");
			}
			command_.setLength(0);
		
		} else if (CommandParseMachine.isEqual(command_, "MOTION_LINRelBase")){
			 
			logger_.info("MOTION_LINRelBase receive");
			try {
				MotionExecuteMachine.LINMotionRelBase();
				this.sendCommand(ack);
				logger_.info("MOTION_LINRelBase done");
			} catch (Exception e){
				this.sendCommand(error);
				logger_.info("MOTION_LINRelBase error");
			}
			command_.setLength(0);
			
		} else if (CommandParseMachine.isEqual(command_, "MOTION_CIRC")){
			 
			logger_.info("MOTION_CIRC receive");
			try {
				MotionExecuteMachine.CIRCMotion();
				this.sendCommand(ack);
				logger_.info("MOTION_CIRC done");
			} catch (Exception e){
				this.sendCommand(error);
				logger_.info("MOTION_CIRC error");
			}
			command_.setLength(0);
		}
	}

	// TODO 考虑如何修改
	private void ResponseUPDATECommand()
	{
		// 响应UPDATE命令：在末端法兰安装工具
		if (CommandParseMachine.isPrefix(command_, "UPDATE_attachTool_"))
		{
			double[] toolInfo = CommandParseMachine.updateToolInformation(command_.toString());

			if (toolInfo.length == 10)
			{
				this.attachToolToFlange("Tool", toolInfo);
				this.sendCommand(ack);
				command_.setLength(0);
				logger_.info("The tool is attached successfully");
			} else {
				this.sendCommand(error);
				command_.setLength(0);
				logger_.info("Could not attach tool to robot, App terminated");
				KUKAServerManager.terminate_flag_ = true;
			}
		}

		// 响应UPDATE命令：设置关节坐标值（普通模式）
		else if (CommandParseMachine.isPrefix(command_, "UPDATE_jPos_")){
			
			if(CommandParseMachine.updateJointPos(command_.toString())){
				this.sendCommand(ack);
			} else {
				this.sendCommand(error);
			}
			command_.setLength(0);
		}
		
		// 响应UPDATE命令：设置关节相对速度：常量值（普通模式）
		else if (CommandParseMachine.isPrefix(command_, "UPDATE_jRelVel_"))
		{
			if (CommandParseMachine.updateJointRelVelocity(command_.toString())){
				this.sendCommand(ack);
			} else {
				this.sendCommand(error);
				}
			command_.setLength(0);
		}	
		
		// 响应UPDATE命令：设置EEF绝对速度：常量值（普通模式）
		else if (CommandParseMachine.isPrefix(command_, "UPDATE_cVel_")){
		
			if (CommandParseMachine.updateCartVelocity(command_.toString())){
				this.sendCommand(ack);
			} else {
				this.sendCommand(error);
			}
			command_.setLength(0);
		}
		
		// 响应UPDATE命令：更新各个关节的速度，7维向量（普通模式）
		else if (CommandParseMachine.isPrefix(command_, "UPDATE_jVel_")){
				
			if (CommandParseMachine.updateJointVel(command_.toString())){
				this.sendCommand(ack);
			} else {
				this.sendCommand(error);
			}
			command_.setLength(0);
		}
			
	
		// 响应UPDATE命令：更新EEF坐标值（普通模式）
		else if (CommandParseMachine.isPrefix(command_, "UPDATE_EEFPos")){
		
			if (CommandParseMachine.isPrefix(command_, "UPDATE_EEFPosCirc1_")){
			
				if (CommandParseMachine.updateEEFPosOnCircMode(command_.toString(), true)){
					this.sendCommand(ack);
				}else {
					this.sendCommand(error);
				}
				command_.setLength(0);
			} else if (CommandParseMachine.isPrefix(command_, "UPDATE_EEFPosCirc2_")){
			
				if (CommandParseMachine.updateEEFPosOnCircMode(command_.toString(), false)){
					this.sendCommand(ack);
				}else {
					this.sendCommand(error);
				}
				command_.setLength(0);	
			} else {
				
				if (CommandParseMachine.updateEEFPosition(command_.toString())){
					this.sendCommand(ack);
				} else {
					this.sendCommand(error);
				}	
				command_.setLength(0);
			}
		}
		
		// 响应UPDATE命令：更新EEF补偿
		else if (CommandParseMachine.isPrefix(command_, "UPDATE_EEFOffset_")){
		
			if (CommandParseMachine.updateEEFOffset(command_.toString())){
				this.sendCommand(ack);
			} else {
				this.sendCommand(error);
			}
			command_.setLength(0);
		}

		// Servo模式
		// 响应UPDATE命令：设置关节坐标值（servo模式）
		else if (CommandParseMachine.isPrefix(command_, "UPDATE_jPosServo_"))
		{
			if(!CommandParseMachine.updateJointPos(command_.toString()))
			{
				// 若发送的指令格式错误，即不符合规定格式，则停止servo运动
				KUKAServerManager.directSmart_ServoMotionFlag_ = false;
			}
			// 快速模式不返回应答ack
			command_.setLength(0);
		}
		// 响应UPDATE命令：更新EEF坐标值（servo模式）
		else if (CommandParseMachine.isPrefix(command_, "UPDATE_EEFPosServo_"))
		{
			if (!CommandParseMachine.updateEEFPosServo(command_.toString())){
				KUKAServerManager.directSmart_ServoMotionFlag_ = false;
			}
			// 快速模式不返回应答ack
			command_.setLength(0);
		}
		
		// 响应UPDATE命令：更新各个关节的速度，7维向量（servo模式）
		else if (CommandParseMachine.isPrefix(command_, "UPDATE_jVelServo_")){
		
			if (!CommandParseMachine.updateJointVel(command_.toString())){
				KUKAServerManager.directSmart_ServoMotionFlag_ = false;
			} 
			// 快速模式不返回应答ack
			command_.setLength(0);
		}
	}

	private void ResponseQUERYCommand()
	{
		// 响应QUERY命令：查询关节位置值
		if (CommandParseMachine.isEqual(command_, "QUERY_JointPos"))
		{
			this.sendCommand(KUKAServerManager.queryRobotStateVariables_.
					queryJointPosition());
			command_.setLength(0);
		}
	}

	public void attachToolToFlange(String toolFrameName, double[] toolInformation)
	{
		// 负载数据，设置为0代表工具质量为0，即没有外加工具
		LoadData loadData = new LoadData();
		loadData.setMass(toolInformation[9]);
		loadData.setCenterOfMass(toolInformation[6],
				toolInformation[7],
				toolInformation[8]);

		KUKAServerManager.tool_ = new Tool(toolFrameName, loadData);
		XyzAbcTransformation trans = XyzAbcTransformation.ofRad(
				toolInformation[0], toolInformation[1],
				toolInformation[2], toolInformation[3],
				toolInformation[4], toolInformation[5]);

		// 为tool添加坐标系
		ObjectFrame aTransformation = KUKAServerManager.tool_.addChildFrame(
				toolFrameName + "(TCP)", trans);
		KUKAServerManager.tool_.setDefaultMotionFrame(aTransformation);

		// Attach tool to the robot
		KUKAServerManager.tool_.attachTo(KUKAServerManager.LBR_Med_.getFlange());
	}

	// respond to a data Acquisition Request
	private void dataAcquisitionRequest()
	{
		// Write output of Media flange
		if (CommandParseMachine.isPrefix(command_, "blueOn"))
		{
			KUKAServerManager.mff_.blueOn();
			this.sendCommand(ack);
			command_.setLength(0);
		} else if (CommandParseMachine.isPrefix(command_, "blueOff"))
		{
			KUKAServerManager.mff_.blueOff();
			this.sendCommand(ack);
			command_.setLength(0);
		} else if (CommandParseMachine.isPrefix(command_, "pin"))
		{
			if (CommandParseMachine.isPrefix(command_, "pin1on"))
			{
				KUKAServerManager.mff_.pin1On();
				this.sendCommand(ack);
				command_.setLength(0);
			} else if (CommandParseMachine.isPrefix(command_, "pin1off"))
			{
				KUKAServerManager.mff_.pin1Off();
				this.sendCommand(ack);
				command_.setLength(0);
			} else if (CommandParseMachine.isPrefix(command_, "pin11on"))
			{
				KUKAServerManager.mff_.pin11On();
				this.sendCommand(ack);
				command_.setLength(0);
			} else if (CommandParseMachine.isPrefix(command_, "pin11off"))
			{
				KUKAServerManager.mff_.pin11Off();
				this.sendCommand(ack);
				command_.setLength(0);
			} else if (CommandParseMachine.isPrefix(command_, "pin2on"))
			{
				KUKAServerManager.mff_.pin2On();
				this.sendCommand(ack);
				command_.setLength(0);
			} else if (CommandParseMachine.isPrefix(command_, "pin2off"))
			{
				KUKAServerManager.mff_.pin2Off();
				this.sendCommand(ack);
				command_.setLength(0);
			} else if (CommandParseMachine.isPrefix(command_, "pin12on"))
			{
				KUKAServerManager.mff_.pin12On();
				this.sendCommand(ack);
				command_.setLength(0);
			} else if (CommandParseMachine.isPrefix(command_, "pin12off"))
			{
				KUKAServerManager.mff_.pin12Off();
				this.sendCommand(ack);
				command_.setLength(0);
			}
		}
		// Read input of Media flange
		else if (CommandParseMachine.isPrefix(command_, "getPin"))
		{
			if (CommandParseMachine.isPrefix(command_, "getPin10")) {
				KUKAServerManager.mff_.getPin10();
				command_.setLength(0);
			} else if (CommandParseMachine.isPrefix(command_, "getPin16")) {
				KUKAServerManager.mff_.getPin16();
				command_.setLength(0);
			} else if (CommandParseMachine.isPrefix(command_, "getPin13")) {
				KUKAServerManager.mff_.getPin13();
				command_.setLength(0);
			} else if (CommandParseMachine.isPrefix(command_, "getPin3")) {
				KUKAServerManager.mff_.getPin3();
				command_.setLength(0);
			} else if (CommandParseMachine.isPrefix(command_, "getPin4")) {
				KUKAServerManager.mff_.getPin4();
				command_.setLength(0);
			}
		}
	}

	// TODO 判断是否需要放在子线程
	public void directServoStartJoints(){
	
		boolean doDebugPrints = false;
		JointPosition initialPosition = new JointPosition(LBR_Med_.getCurrentJointPosition());

		for (int i = 0; i < 7; i++)
			KUKAServerManager.jPos_[i] = initialPosition.get(i);
		
		DirectServo aDirectServoMotion = new DirectServo(initialPosition);
		aDirectServoMotion.setMinimumTrajectoryExecutionTime(40e-3);

		logger_.info("Starting DirectServo motion in position control mode");
		LBR_Med_.moveAsync(aDirectServoMotion);
		logger_.info("Get the runtime of the DirectServo motion");

		IDirectServoRuntime theDirectServoRuntime = aDirectServoMotion.getRuntime();

		JointPosition destination = new JointPosition(LBR_Med_.getJointCount());
		double disp;
		double temp;
		double absDisp;

		try {
			// do a cyclic loop
			// Do some timing...
			// in nanosec

			while (KUKAServerManager.directSmart_ServoMotionFlag_ == true)
			{
				// ///////////////////////////////////////////////////////
				// Insert your code here
				// e.g Visual Servoing or the like
				// Synchronize with the realtime system
				// theDirectServoRuntime.updateWithRealtimeSystem();

				if (doDebugPrints){
					logger_.info("Current fifth joint position " + KUKAServerManager.jPos_[5]);
					logger_.info("Current joint destination " +
							theDirectServoRuntime.getCurrentJointDestination());
				}

				Thread.sleep(1);
				// getLogger().warn(Double.toString(jpos_[0]));
				// getLogger().info(daCommand);
				
				JointPosition currentPos = new JointPosition(
						LBR_Med_.getCurrentJointPosition());

				for (int k = 0; k < destination.getAxisCount(); ++k) {

					double dj = KUKAServerManager.jPos_[k] - currentPos.get(k);
					disp = this.getTheDisplacement(dj);
					temp = currentPos.get(k) + disp;
					absDisp = Math.abs(disp);
					if (absDisp > KUKAServerManager.jDispMax_[k]) {
						KUKAServerManager.jDispMax_[k] = absDisp;
					}
					destination.set(k, temp);
					KUKAServerManager.updateCycleJointPos_[k] = temp;

				}
				theDirectServoRuntime.setDestination(destination);
			}
		} catch (Exception e) {
			logger_.info(e.getLocalizedMessage());
			e.printStackTrace();
			// Print statistics and parameters of the motion
			logger_.info("Simple Cartesian Test \n"
							+ theDirectServoRuntime.toString());

			logger_.info("Stop the DirectServo motion");

		}

		theDirectServoRuntime.stopMotion();
		logger_.info("Stop the DirectServo motion for the stop instruction was sent");
	}

	//求位移
	double getTheDisplacement(double dj) {
		double a = 0.07;
		double b = a * 0.75;
		double exponenet = -Math.pow(dj / b, 2);
		return Math.signum(dj) * a * (1 - Math.exp(exponenet));
	}
	
}
