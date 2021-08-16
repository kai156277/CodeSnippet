#include <qvector.h>
#include <iostream>
#include <random>

using namespace std;

int main(int argc, char *argv[])
{
    int64_t                                n = 10;
    std::random_device                     rd;          //Will be used to obtain a seed for the random number engine
    std::mt19937                           gen(rd());   //Standard mersenne_twister_engine seeded with rd()
    std::uniform_int_distribution<int32_t> dis(0, 10);

    QVector<int> randflags(n);
    //TODO:  可以设置为RANSAC类的参数

    for (int j = 0; j < n; ++j)
    {
        randflags[j] = dis(gen);
        std::cout << randflags[j] << ' ';
    }

    std::cout  << "\nEND!" << std::endl;
    system("pause");
    return 0;
}