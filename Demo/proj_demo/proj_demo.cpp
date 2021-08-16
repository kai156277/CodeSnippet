// proj_demo.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include "pos_proj.h"
#include <proj.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>

int quick_start();

void get_proj_opt();
void get_proj_ell();

int UTM3N_to_WGS84();
int WGS84_3D_to_2D();

int WGS84_to_ECEF();

int WGS84_to_CGCS2000_2D();
int CGCS2000_2D_to_3D();

void pos_cs2cs_fun();

// PJ_COORD long （-180, 180）

// 注意longitude/latitude输出为弧度
#define CGCS2000_3D "EPSG:4479"
#define CGCS2000_2D "EPSG:4490"
#define WGS84_3D "EPSG:4978"
#define WGS84_2D "EPSG:4326"
#define Beijing54 "EPSG:4214"
#define Xian80 "EPSG:4610"
// EPSG:4479 <=> +proj=geocent +ellps=GRS80 +units=m +no_defs <=> CGCS2000 3D
// EPSG:4490 <=> +proj=longlat +ellps=GRS80 +no_defs <=> CGCS2000 2D
// EPSG:4978 <=> +proj=geocent +ellps=WGS84 +units=m +no_defs <=> WGS84 3D
// EPSG:4326 <=> +proj=longlat +ellps=WGS84 +no_defs <=> WGS84 2D
int main(void)
{
    WGS84_3D_to_2D();
    WGS84_to_ECEF();

    PJ_COORD a, b, c;
    pos_proj pj("+proj=utm +zone=32");
    a = proj_coord(proj_torad(12), proj_torad(56), 23, 0);
    b = pj.trans(a, PJ_FWD);
    display_enu_format(b);

    return 0;
}
int WGS84_to_ECEF()
{
    /*
	*	proj: cart
	*	范围：3D
	*	输入：大地坐标(弧度)
	*	输出：ECEF
	*/
    PJ_CONTEXT *C;
    PJ *        P;
    PJ_COORD    a, b, c;
    C = proj_context_create();
    P = proj_create(C, "+proj=cart +ellps=WGS84");
    if (P == nullptr)
    {
        fprintf(stderr, "Oops\n");
        return 1;
    }
    PJ_PROJ_INFO info = proj_pj_info(P);
    printf("name: %s\n", info.id);
    printf("description: %s\n", info.description);
    printf("definition: %s\n", info.definition);
    printf("name: %s\n", info.id);

    // b: long: 120.107545 lat: 36.0061338, 49.305
    a = proj_coord(-2591211.709, 4468718.325, 3728771.312, 0);
    //a = proj_coord(proj_torad(120.107545), proj_torad(36.0061338), 49.305, 0);

    /* transform to UTM zone 32, then back to geographical */
    b = proj_trans(P, PJ_INV, a);
    printf("longitude: %.6f, latitude: %.6f, h: %.6f\n", proj_todeg(b.lpz.lam), proj_todeg(b.lpz.phi), proj_todeg(b.lpz.z));
    c = proj_trans(P, PJ_FWD, b);
    printf("x: %.6f, y: %.6f, z: %.6f\n", c.xyz.x, c.xyz.y, c.xyz.z);

    /* Clean up */
    proj_destroy(P);
    proj_context_destroy(C); /* may be omitted in the single threaded case */
    return 0;
}

int WGS84_to_CGCS2000_2D()
{
    //quick_start();

    PJ_CONTEXT *C;
    PJ *        P;
    PJ *        P_for_GIS;
    PJ_COORD    a, b, c;
    C = proj_context_create();
    P = proj_create_crs_to_crs(C, "EPSG:4326", "EPSG:4490", nullptr);
    if (P == nullptr)
    {
        fprintf(stderr, "Oops\n");
        return 1;
    }

    P_for_GIS = proj_normalize_for_visualization(C, P);
    if (0 == P_for_GIS)
    {
        fprintf(stderr, "Oops\n");
        return 1;
    }
    proj_destroy(P);
    P = P_for_GIS;

    PJ_PROJ_INFO info = proj_pj_info(P);
    printf("name: %s\n", info.id);
    printf("description: %s\n", info.description);
    printf("definition: %s\n", info.definition);
    printf("name: %s\n", info.id);

    // b: long: 120.1075453563 lat: 36.0061344153, h: 49.3047
    a = proj_coord(120.1075453563, 36.0061344153, 49.3047, 0);

    b = proj_trans(P, PJ_FWD, a);
    printf("longitude: %.11f, latitude: %.11f, h: %.4f\n", b.lpz.lam, b.lpz.phi, b.lpz.z);

    c = proj_trans(P, PJ_INV, b);
    printf("longitude: %.11f, latitude: %.11f, h: %.4f\n", c.lpz.lam, c.lpz.phi, c.lpz.z);
    //printf("x: %.4f, y: %.4f, z: %.4f\n", c.xyz.x, c.xyz.y, c.xyz.z);

    /* Clean up */
    proj_destroy(P);
    proj_context_destroy(C); /* may be omitted in the single threaded case */
    return 0;
}

int CGCS2000_2D_to_3D()
{
    PJ_CONTEXT *C;
    PJ *        P;
    PJ *        P_for_GIS;
    PJ_COORD    a, b, c;
    C = proj_context_create();
    P = proj_create_crs_to_crs(C, "EPSG:4479", "EPSG:4490", nullptr);

    P_for_GIS = proj_normalize_for_visualization(C, P);
    if (0 == P_for_GIS)
    {
        fprintf(stderr, "Oops\n");
        return 1;
    }
    proj_destroy(P);
    P = P_for_GIS;

    if (P == nullptr)
    {
        fprintf(stderr, "Oops\n");
        return 1;
    }
    PJ_PROJ_INFO info = proj_pj_info(P);
    printf("name: %s\n", info.id);
    printf("description: %s\n", info.description);
    printf("definition: %s\n", info.definition);
    printf("name: %s\n", info.id);

    a = proj_coord(-2591211.709, 4468718.325, 3728771.312, 0);

    /* transform to UTM zone 32, then back to geographical */
    b = proj_trans(P, PJ_FWD, a);
    char lam[256];
    proj_rtodms(lam, proj_torad(b.lpz.lam), 0, 0);
    printf("longitude: %s\n", lam);
    printf("longitude: %.11f, latitude: %.11f, h: %.4f\n", b.lpz.lam, b.lpz.phi, b.lpz.z);
    // b: long: 120.1075453563 lat: 36.0061344153, h: 49.3047
    c = proj_trans(P, PJ_INV, b);
    printf("x: %.4f, y: %.4f, z: %.4f\n", c.xyz.x, c.xyz.y, c.xyz.z);

    /* Clean up */
    proj_destroy(P);
    proj_context_destroy(C); /* may be omitted in the single threaded case */
    return 0;
    return 0;
}

void pos_cs2cs_fun()
{
    PJ_COORD  a, b, c;
    pos_cs2cs proj_(WGS84_2D, Xian80);
    printf("trans ok: %s\n", (proj_.isValid() ? "Yes" : "No"));
    a = proj_coord(proj_torad(120.1075453563), proj_torad(36.0061344153), 49.3047, 0);
    b = proj_.trans(a, PJ_FWD);

    /* transform to UTM zone 32, then back to geographical */
    printf("longitude: %.11f,"
           "latitude: %.11f,"
           " h: %.4f\n",
           proj_todeg(b.lpz.lam),
           proj_todeg(b.lpz.phi),
           b.lpz.z);
    // b: long: 120.1075453563 lat: 36.0061344153, h: 49.3047
    c = proj_.trans(b, PJ_INV);
    printf("x: %.11f,"
           " y: %.11f,"
           " z: %.11f\n",
           proj_todeg(c.xyz.x),
           proj_todeg(c.xyz.y),
           c.xyz.z);
}

int UTM3N_to_WGS84()
{
    PJ_CONTEXT *C;
    PJ *        P;
    PJ *        P_for_GIS;
    PJ_COORD    a, b, c;

    /* or you may set C=PJ_DEFAULT_CTX if you are sure you will     */
    /* use PJ objects from only one thread                          */
    C = proj_context_create();

    printf("EPSG:32603 <=> EPSG:4326\n");
    P = proj_create_crs_to_crs(C,
                               "EPSG:32603",
                               "EPSG:4326", /* or EPSG:32632 */
                               NULL);

    if (0 == P)
    {
        fprintf(stderr, "Oops\n");
        return 1;
    }

    /* This will ensure that the order of coordinates for the input CRS */
    /* will be longitude, latitude, whereas EPSG:4326 mandates latitude, */
    /* longitude */
    P_for_GIS = proj_normalize_for_visualization(C, P);
    if (0 == P_for_GIS)
    {
        fprintf(stderr, "Oops\n");
        return 1;
    }
    proj_destroy(P);
    P                 = P_for_GIS;
    PJ_PROJ_INFO info = proj_pj_info(P);
    printf("name: %s\n", info.id);
    printf("description: %s\n", info.description);
    printf("definition: %s\n", info.definition);
    printf("name: %s\n", info.id);

    /* a coordinate union representing Copenhagen: 55d N, 12d E    */
    /* Given that we have used proj_normalize_for_visualization(), the order of
    /* coordinates is longitude, latitude, and values are expressed in degrees. */
    a = proj_coord(406353.685304, 6664707.329606, 30, 0);

    /* transform to UTM zone 32, then back to geographical */
    b = proj_trans(P, PJ_FWD, a);
    printf("longitude: %.9f, latitude: %.9f\n", b.lp.lam, b.lp.phi);
    c = proj_trans(P, PJ_INV, b);
    printf("easting: %.6f, northing: %.6f\n", c.enu.e, c.enu.n);

    /* Clean up */
    proj_destroy(P);
    proj_context_destroy(C); /* may be omitted in the single threaded case */
    return 0;
}

int WGS84_3D_to_2D()
{
    //quick_start();

    PJ_CONTEXT *C;
    PJ *        P;
    PJ *        P_for_GIS;
    PJ_COORD    a, b, c;
    C = proj_context_create();
    P = proj_create_crs_to_crs(C, "EPSG:4978", "EPSG:4326", nullptr);

    P_for_GIS = proj_normalize_for_visualization(C, P);
    if (0 == P_for_GIS)
    {
        fprintf(stderr, "Oops\n");
        return 1;
    }
    proj_destroy(P);
    P = P_for_GIS;

    if (P == nullptr)
    {
        fprintf(stderr, "Oops\n");
        return 1;
    }
    PJ_PROJ_INFO info = proj_pj_info(P);
    printf("name: %s\n", info.id);
    printf("description: %s\n", info.description);
    printf("definition: %s\n", info.definition);
    printf("name: %s\n", info.id);

    a = proj_coord(-2591211.709, 4468718.325, 3728771.312, 0);

    /* transform to UTM zone 32, then back to geographical */
    b = proj_trans(P, PJ_FWD, a);
    printf("longitude: %.11f, latitude: %.11f, h: %.4f\n", b.lpz.lam, b.lpz.phi, b.lpz.z);
    // b: long: 120.1075453563 lat: 36.0061344153, h: 49.3047
    c = proj_trans(P, PJ_INV, b);
    printf("x: %.4f, y: %.4f, z: %.4f\n", c.xyz.x, c.xyz.y, c.xyz.z);

    /* Clean up */
    proj_destroy(P);
    proj_context_destroy(C); /* may be omitted in the single threaded case */
    return 0;
}

void get_proj_ell()
{
    const PJ_ELLPS *ell;
    for (ell = proj_list_ellps(); ell->id; ++ell)
    {
        printf("%s\t\t", ell->id);
        printf("%s\t\t", ell->major);
        printf("%s\t\t", ell->ell);
        printf("%s\n", ell->name);
    }
}

void get_proj_opt()
{
    const PJ_OPERATIONS *ops;
    printf("projection keyword: \n");
    for (ops = proj_list_operations(); ops->id; ++ops)
    {
        printf("%s,\t %s\n", ops->id, *ops->descr);
    }
}

int quick_start()
{
    {
        PJ_INFO pj_info = proj_info();
        printf("major: %d, minor: %d, patch: %d\n", pj_info.major, pj_info.minor, pj_info.patch);
        printf("release: %s\n", pj_info.release);
        printf("version: %s\n", pj_info.version);
        printf("search path: %s\n", pj_info.searchpath);
        printf("----------------\n\n");
    }
    PJ_CONTEXT *C;
    PJ *        P;
    PJ *        P_for_GIS;
    PJ_COORD    a, b;

    /* or you may set C=PJ_DEFAULT_CTX if you are sure you will     */
    /* use PJ objects from only one thread                          */
    C = proj_context_create();

    // 设置 proj 的 resource 文件的搜索路径
    // TODO: 加一个测试验证可用
    const char *path = ".\\share\\proj";
    proj_context_set_search_paths(C, 1, &path);
    {
        printf("EPSG:4978-------------\n");
        P = proj_create(C, "EPSG:4978");

        if (P != nullptr)
        {
            PJ_PROJ_INFO info = proj_pj_info(P);

            printf("name: %s\n", info.id);
            printf("description: %s\n", info.description);
            printf("definition: %s\n", info.definition);
        }

        proj_destroy(P);
    }

    {
        printf("+proj=utm +zone=32 +datum=WGS84\n");
        P = proj_create(C, "+proj=utm +zone=32 +datum=WGS84");

        if (P != nullptr)
        {
            PJ_PROJ_INFO info = proj_pj_info(P);

            printf("name: %s\n", info.id);
            printf("description: %s\n", info.description);
            printf("definition: %s\n", info.definition);
        }

        proj_destroy(P);
    }

    {
        printf("EPSG:32632-------------\n");
        P = proj_create(C, "EPSG:32632");

        if (P != nullptr)
        {
            PJ_PROJ_INFO info = proj_pj_info(P);

            printf("name: %s\n", info.id);
            printf("description: %s\n", info.description);
            printf("definition: %s\n", info.definition);
        }

        proj_destroy(P);
    }

    printf("+proj=longlat +datum=WGS84 +no_defs <=> +proj=utm +zone=32 +datum=WGS84\n");
    P = proj_create_crs_to_crs(C,
                               "+proj=longlat +datum=WGS84 +no_defs",
                               "+proj=utm +zone=32 +datum=WGS84", /* or EPSG:32632 */
                               NULL);

    if (0 == P)
    {
        fprintf(stderr, "Oops\n");
        return 1;
    }

    /* This will ensure that the order of coordinates for the input CRS */
    /* will be longitude, latitude, whereas EPSG:4326 mandates latitude, */
    /* longitude */
    P_for_GIS = proj_normalize_for_visualization(C, P);
    if (0 == P_for_GIS)
    {
        fprintf(stderr, "Oops\n");
        return 1;
    }
    proj_destroy(P);
    P = P_for_GIS;

    /* a coordinate union representing Copenhagen: 55d N, 12d E    */
    /* Given that we have used proj_normalize_for_visualization(), the order of
    /* coordinates is longitude, latitude, and values are expressed in degrees. */
    a = proj_coord(12, 55, 0, 0);

    /* transform to UTM zone 32, then back to geographical */
    b = proj_trans(P, PJ_FWD, a);
    printf("easting: %.3f, northing: %.3f\n", b.enu.e, b.enu.n);
    b = proj_trans(P, PJ_INV, b);
    printf("longitude: %g, latitude: %g\n", b.lp.lam, b.lp.phi);

    /* Clean up */
    proj_destroy(P);
    proj_context_destroy(C); /* may be omitted in the single threaded case */
    return 0;
}

// 运行程序: Ctrl + F5 或调试 >“开始执行(不调试)”菜单
// 调试程序: F5 或调试 >“开始调试”菜单

// 入门使用技巧:
//   1. 使用解决方案资源管理器窗口添加/管理文件
//   2. 使用团队资源管理器窗口连接到源代码管理
//   3. 使用输出窗口查看生成输出和其他消息
//   4. 使用错误列表窗口查看错误
//   5. 转到“项目”>“添加新项”以创建新的代码文件，或转到“项目”>“添加现有项”以将现有代码文件添加到项目
//   6. 将来，若要再次打开此项目，请转到“文件”>“打开”>“项目”并选择 .sln 文件
