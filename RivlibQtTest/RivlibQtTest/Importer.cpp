#include "Importer.h"

#include <spdlog/qt_spdlog.h>

using namespace scanlib;

Importer::Importer()
    : pointcloud(true)
    , mLine(0)
    , mFrame(0)
{
}

void Importer::on_echo_transformed(echo_type echo)
{
    const float zero = 0;

    if (pointcloud::none != echo)
    {
        SPDLOG_TRACE("target size: {}", target_count);
        target &t(targets[target_count - 1]);
        double  range = std::sqrt(t.vertex[0] * t.vertex[0] + t.vertex[1] * t.vertex[1] + t.vertex[2] * t.vertex[2]);
        t.time;
    }
}
