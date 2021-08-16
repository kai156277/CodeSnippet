#ifndef SECTIONFITTING_H
#define SECTIONFITTING_H

#include "TunnelDefType.h"

#include <QObject>
#include <QString>
#include <algorithm>

class SectionFitting : public QObject
{
    Q_OBJECT
signals:
    void processNum(int);

public:
    enum Type
    {
        Bspline = 0,
        Bezier,
        None
    };
    static QString TypeToQString(Type type);

    static int algorithmCount();

    bool isProcess();
    void fitting(Type type, const TunnelPointCloud &section, TunnelPointCloud &newvector, int numberofnewpoint);
    // 异步的算法调用，可以通过 processNum 信号获得进度
    void async_fitting(Type type, const TunnelPointCloud &section, TunnelPointCloud &newvector, int numberofnewpoint);

private:
    // 算法的纯粹实现
    void bezier(const TunnelPointCloud &section, TunnelPointCloud &newvector, int numberofnewpoint);
    void bspline(const TunnelPointCloud &section, TunnelPointCloud &newvector, int numberofnewpoint);
    bool process = false;
};

#endif   // SECTIONFITTING_H
