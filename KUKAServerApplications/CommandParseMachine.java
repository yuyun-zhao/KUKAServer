package KUKAServerApplications;

import java.util.StringTokenizer;

public class CommandParseMachine
{
	// 判断string_buffer是否与str相等
	public static boolean isEqual(StringBuffer string_buffer,
								  String str)
	{
		return string_buffer.toString().equals(str);
	}

	// 判断string_buffer是否以str开头
	public static boolean isPrefix(StringBuffer string_buffer,
								   String str)
	{
		return string_buffer.toString().startsWith(str);
	}

	// ---------------------------------------------------------------------------
	//   以下函数用以解析上位机命令
	// ---------------------------------------------------------------------------

	// 返回工具的坐标系数组
	public static double[] updateToolInformation(String cmd)
	{
		double[] toolInfo = { 0., 0., 0., 0., 0., 0., 0., 0., 0., 0. };
		int numericDataCount = 0;

		StringTokenizer st = new StringTokenizer(cmd, "_");
		st.nextToken();
		st.nextToken();

		if (st.countTokens() == 10)
		{
			for (int j = 0; j < 10; j++)
			{
				toolInfo[j] = Double.parseDouble(st.nextToken());
				numericDataCount++;
			}
		}
		
		if (numericDataCount == 10)
		{
			return toolInfo;
		} else {
			return new double[]{ 0.0 };
		}
	}

	// 更新关节值向量
	public static boolean updateJointPos(String command)
	{
		// 按照"_"进行切分
		StringTokenizer st = new StringTokenizer(command, "_");

		// StringTokenizer对象中的下一个token
		// 第一次调用时返回第一个token的内容
		st.nextToken();
		st.nextToken();

		// 判断字符串中是含还有分隔符"_"
		if (st.countTokens() == KUKAServerManager.jNum_)
		{
			for (int j = 0; j < KUKAServerManager.jNum_; j++)
			{
				// TODO: 未来考虑是否需要添加异常处理
				KUKAServerManager.jPos_[j] = Double.parseDouble(st.nextToken());
			}
			return true;

		} else {
			// 若发送的指令格式错误，放回false提醒上位机
			return false;
		}
	}

	// 更新关节相对运动速度
	public static boolean updateJointRelVelocity(String command)
	{
		StringTokenizer st = new StringTokenizer(command, "_");
		st.nextToken();
		st.nextToken();

		if (st.countTokens() == 1){
			KUKAServerManager.jRelVel_ = Double.parseDouble(st.nextToken());
			return true;
		} else {
			return false;
		}
	}
	
	//更新关节相对速度（Servo模式）
	public static boolean updatejRelVelServo(String command)
	{
		StringTokenizer st = new StringTokenizer(command, "_");
		st.nextToken();
		st.nextToken();
		
		if(st.countTokens() == 1){
			try{
				KUKAServerManager.jRelVelServo_ = Double.parseDouble(st.nextToken());
			}catch (Exception e){
				return false;
			}
			return true;
		}else {
			return false;
		}
	}

	// 更新EEF绝对运动速度
	public static boolean updateCartVelocity(String command)
	{
		StringTokenizer st = new StringTokenizer(command, "_");
		st.nextToken();
		st.nextToken();

		if (st.countTokens() == 1)
		{
			KUKAServerManager.cVel_ = Double.parseDouble(st.nextToken());
			return true;
		} else {
			return false;
		}
	}

	// 更新EEF位置（普通模式）
	public static boolean updateEEFPosition(String command)
	{
		StringTokenizer st = new StringTokenizer(command, "_");
		st.nextToken();
		st.nextToken();

		if (st.countTokens() == 6){
			
			for (int j = 0; j < 6; j++)
				KUKAServerManager.EEFPos_[j] = Double.parseDouble(st.nextToken());
			
			return true;
			
		} else {
			return false;
		}
	}

	public static boolean updateEEFOffset(String command)
	{
		StringTokenizer st = new StringTokenizer(command, "_");
		st.nextToken();
		st.nextToken();

		if (st.countTokens() == 3)
		{
			for (int j = 0; j < 3; j++)
			{
				KUKAServerManager.EEFOffset_[j] = Double.parseDouble(st.nextToken());
			}
			return true;

		} else {
			return false;
		}

	}

	// 更新关节速度值向量
	public static boolean updateJointVel(String command)
	{
		StringTokenizer st = new StringTokenizer(command, "_");
		st.nextToken();
		st.nextToken();

		if (st.countTokens() == KUKAServerManager.jNum_)
		{
			for (int j = 0; j < KUKAServerManager.jNum_; j++)
			{
				try {
					KUKAServerManager.jVelArray_[j] = Double.parseDouble(st.nextToken());
				} catch (Exception e) {
					return false;
				}
			}
			return true;

		} else {
			// 若发送的指令格式错误，放回false提醒上位机
			return false;
		}
	}

	// 更新EEF位置（Servo模式）
	public static boolean updateEEFPosServo(String command)
	{
		StringTokenizer st = new StringTokenizer(command, "_");
		st.nextToken();
		st.nextToken();

		if (st.countTokens() == 6)
		{
			for (int j = 0; j < 6; j++)
			{
				try {
					KUKAServerManager.EEFPos_Servo_[j] = Double.parseDouble(st.nextToken());
				} catch (Exception e) {
					return false;
				}
			}
			return true;

		} else {
			return false;
		}
	}
	
	// 应用于圆弧运动模式，更新EEF位置
	// bOne为true时，设置为Circ1；为false时，设置为Circ2
	public static boolean updateEEFPosOnCircMode(String command, boolean bOne)
	{
		StringTokenizer st = new StringTokenizer(command, "_");
		st.nextToken();
		st.nextToken();

		if (st.countTokens() == 6)
		{
			for (int j = 0; j < 6; j++)
			{
				if (bOne)
				{
					KUKAServerManager.EEFPosCIRC1_[j] = Double.parseDouble(st.nextToken());
				} else {
					KUKAServerManager.EEFPosCIRC2_[j] = Double.parseDouble(st.nextToken());
				}
			}
			return true;

		} else {
			return false;
		}

	}


	public static int get_Indexes_ValBoundaries(String daCommand,
												double[] indices, double[] minTorque, double[] maxTorque) {
		StringTokenizer st = new StringTokenizer(daCommand, "_");
		if (st.hasMoreElements())st.nextToken();
		int cnt = 0;
		while (st.hasMoreTokens()) {
			indices[cnt] = Double.parseDouble(st.nextToken());
			minTorque[cnt] = Double.parseDouble(st.nextToken());
			maxTorque[cnt] = Double.parseDouble(st.nextToken());
			++cnt;
		}
		return cnt;
	}
}
