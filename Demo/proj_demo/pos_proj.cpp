#include "pos_proj.h"
#include <proj.h>
#include <string>
#include <vector>
std::string pos_cs2cs::search_path_;
pos_cs2cs::pos_cs2cs(const char *src, const char *dest)
    : pj_context_(proj_context_create())
    , pj_(nullptr)
{
    if (!search_path_.empty())
    {
        const char *path = search_path_.c_str();
        proj_context_set_search_paths(pj_context_, 1, &path);
    }
    PJ *P;
    printf("\n%s <=> %s\n", src, dest);
    P               = proj_create_crs_to_crs(pj_context_, src, dest, nullptr);
    PJ_INFO pj_info = proj_info();
    printf("major: %d, minor: %d, patch: %d\n", pj_info.major, pj_info.minor, pj_info.patch);
    printf("release: %s\n", pj_info.release);
    printf("version: %s\n", pj_info.version);
    printf("search path: %s\n", pj_info.searchpath);
    printf("----------------\n\n");
    if (P == nullptr)
    {
        proj_destroy(P);
        return;
    }

    pj_ = proj_normalize_for_visualization(pj_context_, P);
    if (0 == pj_)
    {
        proj_destroy(P);
        return;
    }
    proj_destroy(P);

    PJ_PROJ_INFO info = proj_pj_info(pj_);
    printf("name: %s\n", info.id);
    printf("description: %s\n", info.description);
    printf("definition: %s\n\n", info.definition);
}

pos_cs2cs::~pos_cs2cs()
{
    proj_destroy(pj_);
    proj_context_destroy(pj_context_);
}

bool pos_cs2cs::isValid() const
{
    return pj_ != nullptr;
}

PJ_COORD pos_cs2cs::trans(const PJ_COORD &src, PJ_DIRECTION direction)
{
    return proj_trans(pj_, direction, src);
}

bool pos_cs2cs::trans(const PJ_COORD &src, PJ_DIRECTION direction, PJ_COORD *dest)
{
    if (dest == nullptr || !isValid())
        return false;
    *dest = proj_trans(pj_, direction, src);
    return true;
}

std::vector<PJ_COORD> pos_cs2cs::trans(const std::vector<PJ_COORD> &src_list, PJ_DIRECTION direction)
{
    std::vector<PJ_COORD> pj_coords;
    pj_coords.reserve(src_list.size());
    for (const auto &src : src_list)
    {
        pj_coords.push_back(std::move(proj_trans(pj_, direction, src)));
    }

    return pj_coords;
}

bool pos_cs2cs::trans(const std::vector<PJ_COORD> &src_list, PJ_DIRECTION direction, std::vector<PJ_COORD> *dest_list)
{
    if (dest_list == nullptr || !isValid())
        return false;

    dest_list->clear();
    dest_list->reserve(src_list.size());
    for (const auto &src : src_list)
    {
        dest_list->push_back(std::move(proj_trans(pj_, direction, src)));
    }
    return true;
}

bool pos_cs2cs::set_search_path(const std::string &search_path)
{
    if (search_path.empty())
        return false;

    auto context = proj_context_create();

    const char *path = search_path.c_str();
    proj_context_set_search_paths(context, 1, &path);
    PJ *P;
    P = proj_create(context, "EPSG:4326");

    if (P == nullptr)
    {
        printf("create faild\n");
        int context_errno = proj_context_errno(context);
        printf("pj errno %d, str: %s\n", context_errno, proj_errno_string(context_errno));
        printf("use defalut search path\n");
        return false;
    }

    search_path_ = search_path;
    return true;
}

pos_proj::pos_proj(const char *proj_string)
    : pj_context_(nullptr)
    , pj_(nullptr)
{
    pj_context_ = proj_context_create();
    pj_         = proj_create(pj_context_, proj_string);
    if (pj_ == nullptr)
    {
        printf("proj create error! \n");
        return;
    }
    PJ_PROJ_INFO info = proj_pj_info(pj_);
    printf("name: %s\n", info.id);
    printf("description: %s\n", info.description);
    printf("definition: %s\n\n", info.definition);
}

pos_proj::~pos_proj()
{
    proj_destroy(pj_);
    proj_context_destroy(pj_context_);
}

PJ_COORD pos_proj::trans(const PJ_COORD &src, PJ_DIRECTION direction)
{
    return proj_trans(pj_, direction, src);
}

void display_lpz_format(const PJ_COORD &src)
{
    printf("longitude: %.11f, latitude: %.11f\n, h: %.11f\n", src.v[0], src.v[1], src.v[2]);
}

void display_xyz_format(const PJ_COORD &src)
{
    printf("x: %.11f, y: %.11f, z: %.11f\n", src.v[0], src.v[1], src.v[2]);
}

void display_enu_format(const PJ_COORD &src)
{
    printf("easting: %.11f, northing: %.11f, h: %.11f\n", src.v[0], src.v[1], src.v[2]);
}
