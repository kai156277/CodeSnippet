// wminfo_test.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include "WMIInfo.h"
#include <iostream>
#include <string>

using namespace std;

int main()
{
	system("chcp 65001");
	CWmiInfo WMI;
	WMI.InitWmi();

	// 网卡原生MAC地址
	CString strNetwork;
	WMI.GetSingleItemInfo(L"Win32_NetworkAdapter WHERE (MACAddress IS NOT NULL) AND (NOT (PNPDeviceID LIKE 'ROOT%'))", L"PNPDeviceID", strNetwork);
	if (!strNetwork.IsEmpty())
	{
		wcout << "Win32_NetworkAdapter: " << strNetwork.GetBuffer() << endl;
	}

	// 硬盘序列号
	CString strDiskDrive;
	WMI.GetSingleItemInfo(L"Win32_DiskDrive WHERE (SerialNumber IS NOT NULL) AND (MediaType LIKE 'Fixed hard disk%')", L"SerialNumber", strDiskDrive);

	if (!strDiskDrive.IsEmpty())
	{
		wcout << "Win32_DiskDrive: " << strDiskDrive.GetBuffer() << endl;
	}

	// 主板序列号
	CString strBaseBoard;
	WMI.GetSingleItemInfo(L"Win32_BaseBoard WHERE (SerialNumber IS NOT NULL)", L"SerialNumber", strBaseBoard);

	if (!strBaseBoard.IsEmpty())
	{
		wcout << "Win32_BaseBoard: " << strBaseBoard.GetBuffer() << endl;
	}

	// 处理器ID
	CString strProcessorID;
	WMI.GetSingleItemInfo(L"Win32_Processor WHERE (ProcessorId IS NOT NULL)", L"ProcessorId", strProcessorID);

	if (!strProcessorID.IsEmpty())
	{
		wcout << "Win32_Processor : " << strProcessorID.GetBuffer() << endl;
	}

	// BIOS序列号
	CString strBIOS;
	WMI.GetSingleItemInfo(L"Win32_BIOS WHERE (SerialNumber IS NOT NULL)", L"SerialNumber", strBIOS);

	if (!strBIOS.IsEmpty())
	{
		wcout << "Win32_BIOS WHERE : " << strBIOS.GetBuffer() << endl;
	}

	// 主板型号
	CString strBaseBoardType;
	WMI.GetSingleItemInfo(L"Win32_BaseBoard WHERE (Product IS NOT NULL)", L"Product", strBaseBoardType);

	if (!strBaseBoardType.IsEmpty())
	{
		wcout << "Win32_BaseBoard : " << strBaseBoardType.GetBuffer() << endl;
	}

	// 网卡当前MAC地址
	CString strCurrentNetwork;
	WMI.GetSingleItemInfo(L"Win32_NetworkAdapter WHERE (MACAddress IS NOT NULL) AND (NOT (PNPDeviceID LIKE 'ROOT%'))", L"MACAddress", strCurrentNetwork);

	if (!strCurrentNetwork.IsEmpty())
	{
		wcout << "Win32_NetworkAdapter: " << strCurrentNetwork.GetBuffer() << endl;
	}
	WMI.ReleaseWmi();

	// 网卡当前MAC地址
	CString strCurrentMemory;
	CString strClassMem[] = { L"PartNumber" };
	WMI.GetGroupItemInfo(L"Win32_PhysicalMemory WHERE (PartNumber IS NOT NULL)", strClassMem, 1, strCurrentMemory);

	if (!strCurrentMemory.IsEmpty())
	{
		wcout << "Win32_NetworkAdapter: " << strCurrentMemory.GetBuffer() << endl;
	}
	WMI.ReleaseWmi();

	return 0;
}

// 运行程序: Ctrl + F5 或调试 >“开始执行(不调试)”菜单
// 调试程序: F5 或调试 >“开始调试”菜单

// 入门使用技巧: 
//   1. 使用解决方案资源管理器窗口添加/管理文件
//   2. 使用团队资源管理器窗口连接到源代码管理
//   3. 使用输出窗口查看生成输出和其他消息
//   4. 使用错误列表窗口查看错误
//   5. 转到“项目”>“添加新项”以创建新的代码文件，或转到“项目”>“添加现有项”以将现有代码文件添加到项目
//   6. 将来，若要再次打开此项目，请转到“文件”>“打开”>“项目”并选择 .sln 文件
