#include <iostream>
#include <vector>
#include <boost/random.hpp>		//随机数所需头文件
using namespace std;
int main(int argc, char **argv)
{
    double mean = 0;
    double stddiv = 0.5;
    boost::mt19937 zgy;								//等分布均匀伪随机数发生器
    zgy.seed(static_cast<unsigned int>(time(0)));	//随机种子
    boost::normal_distribution<> nd(mean, stddiv);	//定义正态分布，均值为mu，标准差为sigma
    boost::variate_generator<boost::mt19937&, boost::normal_distribution<>> gauss_noise(zgy, nd);	//生成高斯噪声
    for(int i = 0;i<30;i++)
    {
        cout<<static_cast<float> (gauss_noise())<<endl;
    }
    return 0;
}

