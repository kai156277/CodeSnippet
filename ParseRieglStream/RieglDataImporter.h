#ifndef RIEGLVZ2000IDATAIMPORTER_H
#define RIEGLVZ2000IDATAIMPORTER_H

#include "riegl/connection.hpp"
#include "riegl/fileconn.hpp"
#include "riegl/ctrllib.hpp"
#include "riegl/ctrlifc.h"
#include "riegl/scanlib.hpp"
#include "riegl/pointcloud.hpp"
#include "riegl/ridataspec.hpp"

#include <QDebug>
#include <QDir>
#include <QTime>
#include <mutex>

class RieglVZ2000iDataImporter : public scanlib::pointcloud
{
public:
    RieglVZ2000iDataImporter();
    ~RieglVZ2000iDataImporter();

    void setSaveDirPath(QString FileName);
protected:
    void on_echo_transformed(echo_type echo) override;
    void on_counter_sync_2angles_hr(const scanlib::counter_sync_2angles_hr<iterator_type>& arg) override; //!< INTERNAL ONLY
    void on_hk_gps_hr(const scanlib::hk_gps_hr<iterator_type> &arg) override;
    void on_unsolicited_message(const scanlib::unsolicited_message<iterator_type> &arg) override;
    void on_pps_synchronized() override;
    void on_pps_sync_lost() override;

private:
    QFile   mParseFile;
    QFile   mExposureFile;
    QFile   mTriggerFile;
};

#endif // RIEGLVZ2000IDATAIMPORTER_H
