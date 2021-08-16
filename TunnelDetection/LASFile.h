#ifndef LASFILE_H
#define LASFILE_H
#include <QString>
class LASFile
{
public:
    LASFile();
    static bool flsToLAS(const QString &fls_file, const QString &las_file);
    static void splitLASFile(const QString &las_file, const QString &las_out_root, int split);
};

#endif   // LASFILE_H
