#include <Kinect.h>
#include <Windows.h>
#include <vector>
#include <algorithm>

using namespace std;

#ifndef _PUBLIC_H_
#define _PUBLIC_H_

typedef vector<POINT> DRAWLIST;
typedef vector<Joint> JOINTS;
typedef vector<JOINTS> JOINTSLIST;
typedef vector<POINT>::iterator DRAWITE;
typedef vector<Joint>::iterator JOINTP;

class Console //����̨��
{
	private:
	HANDLE outhandle;
	HANDLE inhandle;

	public:
	bool succeed;
	Console() : succeed(false)
	{
		succeed = AllocConsole();
		if (succeed)
		{
			outhandle = GetStdHandle(STD_OUTPUT_HANDLE);
			inhandle = GetStdHandle(STD_INPUT_HANDLE);
		}
	}
	~Console()
	{
		FreeConsole();
	}
	DWORD write(string content) //�ڿ���̨���һ���ַ���
	{
		if (!succeed) return -1;
		DWORD _result = 0;
		WriteConsoleA(outhandle, content.c_str(), content.length(), &_result, NULL);
		return _result;
	}
	void pause()
	{
		system("pause");
	}
};

#endif