#include <iomanip>
#include <iostream>
#include <time.h>
#include <windows.h>

#include <BaseProcThread/ScanParam.h>
#include <BaseProcThread/ScanTransThread.h>

using namespace std;

int main(int argc, char *argv[])
{
    ScanTransThread   trans_test;
    xstype::ScanParam scan_param;
    trans_test.Init(&scan_param);
    trans_test.start();
    trans_test.wait();
    cout << "All done!" << endl;

    system("pause");

    return 0;
}
