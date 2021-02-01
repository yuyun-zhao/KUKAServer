package KUKAServerApplications;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.concurrent.atomic.AtomicBoolean;
import com.kuka.med.deviceModel.LBRMed;
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
	
	// 原子Boolean，线程安全地设置_handGuiding_endFlag的布尔值
	public static volatile boolean handguiding_endflag_ = false;

	// ‘\n’的ASCII码为10
	private static final String stopCharacter = Character.toString((char)(10));
	
	// 将"done\n"指令发送给上位机，表明下位机已收到指令，ack：acknowledgement
	private static final String ack = "done" + stopCharacter;
	private static final String error = "error" + stopCharacter;

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
						KUKAServerManager.directServoMotionFlag_ = false;
						KUKAServerManager.terminate_flag_ = true;
						command_.setLength(0);
						break;
					}
					// 开启servo模式
					else if (CommandParseMachine.isEqual(command_, "startServoMode")) {
						try{						
						logger_.info("start direct servo mode");
						KUKAServerManager.servoThread_ = true;
						
						this.sendCommand(ack);
						command_.setLength(0);
						}catch(Exception e){
							this.sendCommand(error);
							logger_.info(e.getMessage());
						}
					}
					// 关闭servo模式
					else if (CommandParseMachine.isEqual(command_, "endServoMode"))
					{
						KUKAServerManager.directServoMotionFlag_ = false;
						logger_.info("direct servo mode terminated");
						this.sendCommand(ack);
						command_.setLength(0);
					}
					// 执行handGuiding
					else if (CommandParseMachine.isEqual(command_, "startHandGuiding"))
					{
						// 运行HandGuiding程序
						try{
							HandGuidingMode hg = new HandGuidingMode(LBR_Med_,logger_);
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
						handguiding_endflag_= true;
						logger_.info(String.valueOf(handguiding_endflag_));
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
//				logger_.info(Double.toString(KUKAServerManager.cVel_));
//				logger_.info(Double.toString(KUKAServerManager.EEFOffset_[1]));
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

		// 响应UPDATE命令：更新EEF坐标值（servo模式）
		else if (CommandParseMachine.isPrefix(command_, "UPDATE_ServoPos_"))
		{	logger_.info("fuck");
			if (CommandParseMachine.updateEEFPosServo(command_.toString())){
				KUKAServerManager.directServoMotionFlag_ = true;
				logger_.info(Double.toString(KUKAServerManager.EEFPos_Servo_[0]));
			}else{
				KUKAServerManager.directServoMotionFlag_ = false;
			}
			// 快速模式不返回应答ack
			command_.setLength(0);
		}
		
		//响应UPDATE命令：更新关节相对速度（servo模式）
		else if (CommandParseMachine.isPrefix(command_, "UPDATE_jRelVelServo_")){
			logger_.info("ddddd");
			if(CommandParseMachine.updatejRelVelServo(command_.toString())){
				logger_.info(Double.toString(KUKAServerManager.jRelVelServo_));
			}else{
				KUKAServerManager.directServoMotionFlag_ = false;
			}
			command_.setLength(0);
		}
	}

	private void ResponseQUERYCommand()
	{
		// 响应QUERY命令：查询关节位置值
		if (CommandParseMachine.isEqual(command_, "QUERY_JointPosition"))
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
	
}
