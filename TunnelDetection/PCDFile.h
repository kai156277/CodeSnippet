#ifndef PCDFILE_H
#define PCDFILE_H

#include <QString>

class PCDFile
{
public:
    PCDFile();

    static bool flsToPCD(const QString &fls_file, const QString &pcd_file);
    static void splitPCDFile(const QString &pcd_file, const QString &pcd_out_root, int split);
};

#endif   // PCDFILE_H
