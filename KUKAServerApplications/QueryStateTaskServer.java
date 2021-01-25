package KUKAServerApplications;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.net.ServerSocket;
import java.net.Socket;

// 该类作用：持续向上位机发送当前机器人信息，如关节位置，关节力矩，外力，法兰位置等
class QueryStateTaskServer implements Runnable
{
	private final int _sendStatePort;//final可理解为恒定量
	private final int _timeout;
	private Socket socket;
	private ServerSocket server_socket;
	private PrintWriter pw_outputStream;
	private BufferedReader br_inputStream;
	private final StringBuffer _commandForState = new StringBuffer();

	QueryStateTaskServer(int sendStatePort, int timeout)//初始化
	{
		_timeout = timeout;//单位为毫秒
		_sendStatePort = sendStatePort;
	}
	
	public void sendCommand(String command_str)//  发送消息指令（String）
	{
		pw_outputStream.write(command_str);
		pw_outputStream.flush();//刷新，把数据刷到上位机去
	}
	
	public void start()
	{
		Thread t_send = new Thread(this);
		t_send.setDaemon(true); // 设置t_send为守护线程，守护线程必须在start之前设置，否则会报错。
		t_send.start();
	}

	@Override
	public void run()
	{
		try {
			server_socket = new ServerSocket(_sendStatePort);
			// 阻塞接收数据，超时后关闭socket并退出程序
				try {
					// 设置超时时间，在_timeout时间内接收不到信息就断开该socket
					server_socket.setSoTimeout(_timeout);
					socket = server_socket.accept();
				} catch (Exception e) {
					// 超时后关闭socket并直接返回
					socket.close();
					server_socket.close();
					KUKAServerManager.terminate_flag_ = true;
					return;
				}

			// 获取输入输出流
				try {
					// getOutputStream: 向客户端发消息，只能写字节数据，不能直接写字符数据
					// getInputStream: 从客户端接收数据，只能读取字节数据，不能直接读字符数据
					// PrintWriter比BufferWriter更实用，可以直接通过outputStream构造，可以直接写字符数据
					pw_outputStream = new PrintWriter(socket.getOutputStream());
				
					// BufferWriter只能先将outputStream先转成outputStreamReader
					// BufferedReader只能先将inputStream先转成inputStreamReader，可以直接读字符数据
					br_inputStream = new BufferedReader(new InputStreamReader(
							socket.getInputStream()));
				} catch (IOException e) {
					System.out.println("Error creating inPort and outPort");
				}

			KUKAServerManager.send_thread_ = true; // send thread began

			if ((socket.isConnected()))
			{
				String msg = null;

				// 程序会一直阻塞在readLine()，等待接收新的指令
				// readLine()方法会判断行末是否有\r或\n，若没有，会一直阻塞，因此必须加上\n
				// readLine()读取到的内容为空时，并不会返回null，而是会一直阻塞，
				// 只有当读取的输入流发生错误或者被关闭时，readLine()方法才会返回null。
				while ((msg = br_inputStream.readLine()) != null)
				{
					// 清空当前StringBuffer，以存储新的指令
					_commandForState.setLength(0);
					_commandForState.append(msg);

					// 查询从客户端发来的信息，判断是哪种指令
					if (CommandParseMachine.isEqual(_commandForState, "END"))
					{
						_commandForState.setLength(0);
						break;
					}
					// Get positions of joints
					else if (CommandParseMachine.isEqual(_commandForState, "QUERY_JointPosition"))
					{
						this.sendCommand(KUKAServerManager.queryRobotStateVariables_.
								queryJointPosition());
						_commandForState.setLength(0);
					}
					// Get torques of joints
					else if (CommandParseMachine.isPrefix(_commandForState, "QUERY_Torques"))
					{
						if (CommandParseMachine.isEqual(_commandForState, "QUERY_Torques_ext_J"))
						{
							this.sendCommand(KUKAServerManager.queryRobotStateVariables_.
									queryJointExternalTorques());
							_commandForState.setLength(0);
						} else if (CommandParseMachine.isEqual(_commandForState, "QUERY_Torques_m_J"))
						{
							this.sendCommand(KUKAServerManager.queryRobotStateVariables_.
									queryJointMeasuredTorques());
							_commandForState.setLength(0);
						}
					}
					// Get parameters of end effector
					else if (CommandParseMachine.isPrefix(_commandForState, "QUERY_EEF"))
					{
						if (CommandParseMachine.isEqual(_commandForState, "QUERY_EEF_force"))
						{
							this.sendCommand(KUKAServerManager.queryRobotStateVariables_.
									queryEEFforces());
							_commandForState.setLength(0);
						} else if (CommandParseMachine.isEqual(_commandForState, "QUERY_EEF_moment"))
						{
							this.sendCommand(KUKAServerManager.queryRobotStateVariables_.
									queryEEFMoments());
							_commandForState.setLength(0);
						} else if (CommandParseMachine.isEqual(_commandForState, "QUERY_EEF_pos"))
						{
							this.sendCommand(KUKAServerManager.queryRobotStateVariables_.
									queryEEFPosition());
							_commandForState.setLength(0);
						}
					} 
				}
			}

			// 当读取的输入流被关闭时，结束上述循环，尝试关闭socket
				try {
					br_inputStream.close();
					pw_outputStream.close();
					socket.close();
					server_socket.close();
					KUKAServerManager.terminate_flag_ = true;
				} catch (IOException e) {
					e.printStackTrace();
				}

		} catch (IOException e1) {
			// 上述非try包含的过程抛出异常时，关闭socket
			try {br_inputStream.close();} 
			catch (IOException e) {e.printStackTrace();}
		
			try {pw_outputStream.close();} 
			catch (Exception e) {e.printStackTrace();}
			
			try {socket.close();}
			catch (IOException e) {e.printStackTrace();}
				
			try {server_socket.close();}
			catch (IOException e) {e.printStackTrace();}

			KUKAServerManager.terminate_flag_ = true;
			e1.printStackTrace();
		}
	}
}
