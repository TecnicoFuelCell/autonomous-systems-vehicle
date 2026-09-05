// NMEA sentence parsing and WGS84 geodetic helpers for gps_pub.
//
// Pure, dependency-free codec (no ROS includes) so it can be unit-tested
// standalone. All functions are inline because the file is header-only.
//
// Semantics preserved from the original single-file gps_node:
//   - malformed numeric fields drop the sentence (parseRMC) or return a
//     fresh Gga{have=false} (parseGGA);
//   - the ENU frame is the standard local ENU of the ROS1-style
//     geodetic -> ECEF -> ENU chain used by /gps_pose;
//   - a missing NMEA checksum is accepted, a wrong one is rejected.
#ifndef GPS_PUB__NMEA_HPP_
#define GPS_PUB__NMEA_HPP_

#include <cmath>
#include <cstddef>
#include <sstream>
#include <string>
#include <vector>

namespace gps_pub {
namespace codec {

// --------------------------- String helpers ---------------------------

inline std::string trim(const std::string &s) {
  auto b = s.find_first_not_of(" \r\n\t");
  auto e = s.find_last_not_of(" \r\n\t");
  if (b == std::string::npos) return "";
  return s.substr(b, e - b + 1);
}

inline std::vector<std::string> split(const std::string &s, char delim = ',') {
  std::vector<std::string> out;
  std::string tok;
  std::istringstream ss(s);
  while (std::getline(ss, tok, delim)) out.push_back(tok);
  return out;
}

// "ddmm.mmmm" + hemisphere -> decimal degrees. N/S/S/W negate.
inline bool ddmm_to_deg(const std::string &ddmm, char hemi, double &out_deg) {
  if (ddmm.empty()) return false;
  auto dot = ddmm.find('.');
  if (dot == std::string::npos || dot < 2) return false;
  int mm_start = static_cast<int>(dot) - 2;
  try {
    double deg = std::stod(ddmm.substr(0, mm_start));
    double minutes = std::stod(ddmm.substr(mm_start));
    double dec = deg + minutes / 60.0;
    if (hemi == 'S' || hemi == 'W') dec = -dec;
    out_deg = dec;
    return true;
  } catch (const std::exception &) {
    return false;  // malformed numeric field -- drop this sentence
  }
}

// XOR of all chars between '$' and '*' must equal the two hex digits after
// '*'. True when no '*' is present at all (some receivers omit it).
inline bool nmea_checksum_ok(const std::string &line) {
  auto star = line.rfind('*');
  if (star == std::string::npos) return true;  // no checksum field present
  if (star + 2 >= line.size()) return false;   // '*' but missing the 2 hex digits
  unsigned char sum = 0;
  for (std::size_t i = 1; i < star; ++i) sum ^= static_cast<unsigned char>(line[i]);
  try {
    unsigned int given = std::stoul(line.substr(star + 1, 2), nullptr, 16);
    return sum == given;
  } catch (const std::exception &) {
    return false;
  }
}

// ------------------------------- Parsing -------------------------------

struct Rmc {
  bool valid = false;
  double lat = std::nan("");
  double lon = std::nan("");
  double speed_kn = 0.0;
  double course_deg = std::nan("");  // A/V validity flag
};

struct Gga {
  bool have = false;
  int fixq = 0;
  int nsat = 0;
  double hdop = 0.0;
  double alt_m = std::nan("");
};

inline Rmc parseRMC(const std::string &line) {
  Rmc r;
  if (line.size() < 6 || line.substr(3, 3) != "RMC") return r;
  auto p = split(line, ',');
  if (p.size() < 10) return r;
  // $GxRMC,1:time,2:status(A/V),3:lat,4:N/S,5:lon,6:E/W,7:speed(kn),8:course,9:date,...
  if (p[2] != "A") return r;  // not valid
  double lat, lon;
  if (!ddmm_to_deg(p[3], p[4].empty() ? 'N' : p[4][0], lat)) return r;
  if (!ddmm_to_deg(p[5], p[6].empty() ? 'E' : p[6][0], lon)) return r;
  try {
    r.speed_kn = p[7].empty() ? 0.0 : std::stod(p[7]);
    r.course_deg = p[8].empty() ? std::nan("") : std::stod(p[8]);
  } catch (const std::exception &) {
    return r;  // malformed speed/course -- leave r.valid false, drop sentence
  }
  r.valid = true;
  r.lat = lat;
  r.lon = lon;
  return r;
}

inline Gga parseGGA(const std::string &line) {
  Gga g;
  if (line.size() < 6 || line.substr(3, 3) != "GGA") return g;
  auto p = split(line, ',');
  if (p.size() < 10) return g;
  // $GxGGA,time,lat,N,lon,E,fix,nsat,hdop,alt,M,...
  try {
    g.fixq = p[6].empty() ? 0 : std::stoi(p[6]);
    g.nsat = p[7].empty() ? 0 : std::stoi(p[7]);
    g.hdop = p[8].empty() ? 0.0 : std::stod(p[8]);
    g.alt_m = p[9].empty() ? std::nan("") : std::stod(p[9]);
  } catch (const std::exception &) {
    return Gga{};  // malformed field -- return a fresh (have=false) struct
  }
  g.have = true;
  return g;
}

// ------------------------------ Geodetics ------------------------------

// WGS84
constexpr double kWgs84A = 6378137.0;
constexpr double kWgs84F = 1.0 / 298.257223563;
constexpr double kWgs84ESq = kWgs84F * (2.0 - kWgs84F);

inline double deg2rad(double degrees) { return degrees * M_PI / 180.0; }

struct Ecef {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

// Geodetic (lat, lon, alt in meters, WGS84) -> Earth-centered, Earth-fixed.
inline Ecef geodeticToEcef(double lat_deg, double lon_deg, double alt_m) {
  const double lat_rad = deg2rad(lat_deg);
  const double lon_rad = deg2rad(lon_deg);
  const double sin_lat = std::sin(lat_rad);
  const double cos_lat = std::cos(lat_rad);
  const double sin_lon = std::sin(lon_rad);
  const double cos_lon = std::cos(lon_rad);
  const double n = kWgs84A / std::sqrt(1.0 - kWgs84ESq * sin_lat * sin_lat);

  Ecef ecef;
  ecef.x = (n + alt_m) * cos_lat * cos_lon;
  ecef.y = (n + alt_m) * cos_lat * sin_lon;
  ecef.z = (n * (1.0 - kWgs84ESq) + alt_m) * sin_lat;
  return ecef;
}

// Local ENU (x = East, y = North, z = Up) offset, in meters, of a geodetic
// point relative to a fixed geodetic origin. Matches the frame used by the
// /gps_pose publisher.
struct Enu {
  double e = 0.0;
  double n = 0.0;
  double u = 0.0;
};

inline Enu enu_from_origin(double origin_lat_deg, double origin_lon_deg,
                           double origin_alt_m, double lat_deg, double lon_deg,
                           double alt_m) {
  const Ecef current = geodeticToEcef(lat_deg, lon_deg, alt_m);
  const Ecef origin = geodeticToEcef(origin_lat_deg, origin_lon_deg, origin_alt_m);
  const double dx = current.x - origin.x;
  const double dy = current.y - origin.y;
  const double dz = current.z - origin.z;

  const double sin_lat = std::sin(deg2rad(origin_lat_deg));
  const double cos_lat = std::cos(deg2rad(origin_lat_deg));
  const double sin_lon = std::sin(deg2rad(origin_lon_deg));
  const double cos_lon = std::cos(deg2rad(origin_lon_deg));

  Enu enu;
  enu.e = -sin_lon * dx + cos_lon * dy;
  enu.n = -sin_lat * cos_lon * dx - sin_lat * sin_lon * dy + cos_lat * dz;
  enu.u = cos_lat * cos_lon * dx + cos_lat * sin_lon * dy + sin_lat * dz;
  return enu;
}

}  // namespace codec
}  // namespace gps_pub

#endif  // GPS_PUB__NMEA_HPP_