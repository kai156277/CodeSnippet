#pragma once
#include <riegl/scanlib.hpp>

class Importer : public scanlib::pointcloud
{
public:
    Importer();

    void on_echo_transformed(echo_type echo) override;

private:
    uint32_t mLine;
    uint32_t mFrame;
};
