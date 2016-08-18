#ifndef __RoboteqDevice_H_
#define __RoboteqDevice_H_

using namespace std;

string ReplaceString(string source, string find, string replacement);
void sleepms(int milliseconds);

class RoboteqDevice
{
private:
	int device_fd;
	int fd0;
	int handle;

protected:
	void InitPort();

	int Write(string str);
	int ReadAll(string &str);

	int IssueCommand(string commandType, string command, string args, int waitms, string &response, bool isplusminus = false);
	int IssueCommand(string commandType, string command, int waitms, string &response, bool isplusminus = false);

public:
	bool IsConnected();
	int Connect(string port);
	void Disconnect();

	int SetConfig(bool CAN, int configItem, int index, int value);
	int SetConfig(bool CAN, int configItem, int value);

	int SetCommand(bool CAN, int commandItem, int index, int value);
	int SetCommand(bool CAN, int commandItem, int value);
	int SetCommand(bool CAN, int commandItem);

	int GetConfig(bool CAN, int configItem, int index, int &result);
	int GetConfig(bool CAN, int configItem, int &result);

	int GetValue(bool CAN, int operatingItem, int index, int &result);
	int GetValue(bool CAN, int operatingItem, int &result);

	RoboteqDevice();
	~RoboteqDevice();
};

#endif
