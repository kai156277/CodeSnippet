#pragma once
#include <proj.h>
#include <string>
#include <vector>
class pos_cs2cs
{
public:
    pos_cs2cs(const char *src, const char *dest);
    ~pos_cs2cs();

    bool isValid() const;

    PJ_COORD trans(const PJ_COORD &src, PJ_DIRECTION direction);
    bool     trans(const PJ_COORD &src, PJ_DIRECTION direction, PJ_COORD *dest);

    std::vector<PJ_COORD> trans(const std::vector<PJ_COORD> &src_list, PJ_DIRECTION direction);
    bool                  trans(const std::vector<PJ_COORD> &src_list, PJ_DIRECTION direction, std::vector<PJ_COORD> *dest_list);

    static bool set_search_path(const std::string &search_path);

private:
    PJ_CONTEXT *pj_context_;
    PJ *        pj_;
    std::string src_crs_;
    std::string dest_crs_;

    static std::string search_path_;
};

class pos_proj
{
public:
    pos_proj(const char *proj_string);
    ~pos_proj();

    PJ_COORD trans(const PJ_COORD &src, PJ_DIRECTION direction);

private:
    PJ_CONTEXT *pj_context_;
    PJ *        pj_;
    std::string proj_string;
};

void display_lpz_format(const PJ_COORD &src);
void display_xyz_format(const PJ_COORD &src);
void display_enu_format(const PJ_COORD &src);
