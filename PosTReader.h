#ifndef PosTReader_H
#define PosTReader_H

#include <string>
 
#include <QDate>

#include <vector>
#include <fstream>
#include <stdint.h>
#include "pubfun.h"
#include "DataType.h"

#include "PosReader.h"
namespace xstype{
    //!  post格式POS读取
    class PosTReader : public PosReader
    {
    public:
        const static int32_t POS_BINARY_VERSION = 1;
        //!  检测文件是否有效
        static bool isValid(const std::string& posFileName){
            uint64_t fsz = xscommon::getFileSize(posFileName); 
        return (fsz > sizeof(PosFileT) * 2) && ((fsz-4) % sizeof(PosFileT) == 0); }
        //!  读取文件开始、结束时间及经度
        bool readFileTime(int64_t fsz, QDate& date, double& minTime, double& maxTime, double& longit)
        { 
            double spanLastTime = -1;
            SpanData spanData;
            int16_t y = 0, m = 0, d = 0;
            while (!atEnd())
            {

                readLine();

                if (!isValid())
                {
                    continue;
                }

                spanData.spanTime = getTime();
                if (spanData.spanTime < 0)
                {
                    continue;
                }
                getBLH(spanData.latitude, spanData.longitude, spanData.heigt);
                longit = spanData.longitude;
                getMDY(y, m, d);
                spanLastTime = spanData.spanTime;
                break;

            }
            if (atEnd())
            { 
                return false;
            }
            date.setDate(y, m, d);
            minTime = spanLastTime;
            seek(fsz - 2 * sizeof(PosFileT));
            while (!atEnd())
            {
                readLine();

                if (!isValid())
                {
                    continue;
                }
            
                spanData.spanTime = getTime();
                if (spanData.spanTime < 0)
                {
                    continue;
                }
                spanLastTime = spanData.spanTime;

            }
            maxTime = spanLastTime;

            return true;
        }
        //!  打开文件
        bool openFile(const std::string& posFileName){
            lastTime = -1;
            spanInput.open(posFileName , std::ios::in|std::ios::binary);
            return spanInput.is_open();
        }
        //!  获取行数
        unsigned getLineNum(int64_t fsz)const { return (unsigned)(fsz / sizeof(PosFileT)); }
        //!  是否到结尾
        bool atEnd() const{ return spanInput.eof(); }
        //!  关闭文件
        void closeFile(){ spanInput.close(); }
        //!  跳过头部
        bool skipHedaer(){
            nVer = 0;
            spanInput.read((char*)&nVer, sizeof(int32_t));
            if (nVer < POS_BINARY_VERSION)
            {
                return false;
            }
            return true;
        }
        //!  读取一行
        void readLine(){
            spanInput.read((char*)&strSpanLine, sizeof(PosFileT));
           
        }
        //!  获取当前行
        const PosFileT& getLine()const{ return strSpanLine; }
        //!  当前行信息转成PosFileT格式
        void getOneLine(PosFileT& post)const
        {
            post = strSpanLine;
        }
        //!  当前行是否合法
        bool isValid()const{
            return spanInput.good();
        }
        //!  获取当前行年月日
        bool getMDY(int16_t& /*year*/, int16_t& /*month*/, int16_t& /*day*/)const {
            return false;
        }
        //!  获取当前行时间
        double getTime()const {
            return strSpanLine.time; 
        }
        //!  获取当前行时分秒
        void getTime(int16_t&hh, int16_t&mm, double& ss)const{
            hh = static_cast<int16_t>(strSpanLine.time * pcm::R_3600);// 
            mm = static_cast<int16_t>(strSpanLine.time * pcm::R_60 - hh * 60.0);
            ss = strSpanLine.time - hh * 3600.0 - mm * 60.0;
        }
        //!  获取当前行空间直角坐标
        bool getXYZ(double&x, double&y, double& z)const{ 
            x = strSpanLine.xyz[0]; y = strSpanLine.xyz[1]; z = strSpanLine.xyz[2];
            return true;
        }
        //!  获取当前行坐标字符串
        std::string getXYZ()const{
            return std::string();
        }
        //!  获取当前行经纬度
        void getBLH(double& latitude, double& longtitude, double& height)const {
            latitude = strSpanLine.blh[0] * RAD_TO_DEG;
            longtitude = strSpanLine.blh[1] * RAD_TO_DEG;
            height = strSpanLine.blh[2];
        }
        //!  获取当前行经纬度 弧度
        void getBLH_RAD(double& latitude, double& longtitude, double& height)const {
            latitude = strSpanLine.blh[0];
            longtitude = strSpanLine.blh[1];
            height = strSpanLine.blh[2];
        }
        //!  获取当前行姿态角
        void getRPH(double& roll, double& pitch, double& heading)const
        {
            roll = strSpanLine.roll;
            pitch = strSpanLine.pitch;
            heading = strSpanLine.heading;
        }
        //!  获取当前行Q值
        int getQ()const{
            return (int)strSpanLine.q;
        }
        //!  获取当前行升沉
        double getHeave()const{ 
            return 0;
        }
        //!  定位某一行
        void seek(size_t pos){ spanInput.seekg(pos); }
        //!  是否二进制格式
        bool isBinary(unsigned& lsz)const { lsz = sizeof(PosFileT); return true; }
    private:
        int32_t nVer;
        PosFileT strSpanLine; 
        std::ifstream spanInput;

        double lastTime;
    };
}
#endif