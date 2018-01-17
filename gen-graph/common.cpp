#include <GeographicLib/Geodesic.hpp>

#include "common.h"
#include "mpreal.h"

GeographicLib::Geodesic geodesic(EARTH_RADIUS, 1/INVERSE_FLATTENING);

mpfr::mpreal calc_distance(mpfr::mpreal lat1, mpfr::mpreal lon1, mpfr::mpreal lat2, mpfr::mpreal lon2) {
    double dist;
    geodesic.Inverse(lat1.toDouble(), lon1.toDouble(), lat2.toDouble(), lon2.toDouble(), dist);
    return mpfr::mpreal(dist);
}

std::string str(mpfr::mpreal mpr) {
    return mpr.toString("%.20RNf");
}

mpfr::mpreal mpreal_round(mpfr::mpreal mpr) {
    if (mpr != MAX_DOUBLE && mpr != -MAX_DOUBLE) {
        mpfr::mpreal ret = str(mpr).c_str();
        return ret;
    }
    return mpr;
}