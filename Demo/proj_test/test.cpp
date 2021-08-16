#include "pch.h"
#include <proj.h>
#include <stdio.h>

class ProjTest : public ::testing::Test
{
public:
};

TEST_F(ProjTest, proj_quick_start)
{
    PJ_CONTEXT *C;
    PJ *        P;
    PJ *        P_for_GIS;
    PJ_COORD    a, b, c;

    C = proj_context_create();
    P = proj_create_crs_to_crs(C,
                               "EPSG:4326",    // WGS84
                               "EPSG:32651",   //
                               //"+proj=utm +zone=51 +datum=WGS84",
                               nullptr);

    /* This will ensure that the order of coordinates for the input CRS */
    /* will be longitude, latitude, whereas EPSG:4326 mandates latitude, */
    /* longitude */
    P_for_GIS = proj_normalize_for_visualization(C, P);
    EXPECT_NE(nullptr, P_for_GIS);
    proj_destroy(P);
    P = P_for_GIS;

    /* a coordinate union representing Copenhagen: 55d N, 12d E    */
    /* Given that we have used proj_normalize_for_visualization(), the order of
    /* coordinates is longitude, latitude, and values are expressed in degrees. */
    a = proj_coord(120.1075, 36.0061112, 0, 0);

    /* transform to UTM zone 32, then back to geographical */
    b = proj_trans(P, PJ_FWD, a);
    printf("easting: %.6f, northing: %.6f\n", b.enu.e, b.enu.n);
    EXPECT_NEAR(b.enu.e, 239291.480, 0.001);
    EXPECT_NEAR(b.enu.n, 3988496.830, 0.001);
    c = proj_trans(P, PJ_INV, b);
    EXPECT_DOUBLE_EQ(c.lp.lam, a.lp.lam);
    EXPECT_DOUBLE_EQ(c.lp.phi, a.lp.phi);
    printf("longitude: %.6f, latitude: %.6f\n", c.lp.lam, c.lp.phi);

    /* Clean up */
    proj_destroy(P);
    proj_context_destroy(C); /* may be omitted in the single threaded case */
    /*
    //b = proj_coord(239291.48, 3988496.83, 0, 0);
    a = proj_coord(12, 55, 0, 0);
    printf("easting: %.6lf, northing: %.6lf\n", a.lpz.lam, a.lpz.phi);
    b = proj_trans(P, PJ_FWD, a);

    printf("easting: %.6lf, northing: %.6lf\n", b.enu.e, b.enu.n);
	*/
}
