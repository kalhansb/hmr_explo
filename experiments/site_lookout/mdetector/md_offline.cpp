// Offline driver for M-detector's DynObjFilter (replaces the ROS node
// dynfilter_with_odom.cpp: same calls, frames from a file instead of topics).
// Parameters come from the ROS parameter server (rosparam load <yaml>).
// Input file: repeated frames of
//   double t; double R[9] (row-major, sensor to world); double p[3]; int32 n; float xyz[3n] (sensor frame)
// Output: <out>/NNNNNN.label (frame-out, after M-detector's clustering) and
// <out>/NNNNNN_o.label (point-out), one int32 per input point: 251 moving, 9 not.
#include <ros/ros.h>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <types.h>
#include <m-detector/DynObjFilter.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "md_offline");
    if (argc < 3) { std::cerr << "usage: md_offline <frames.bin> <out dir>\n"; return 1; }
    ros::NodeHandle nh;
    std::shared_ptr<DynObjFilter> F(new DynObjFilter());
    F->init(nh);
    std::ifstream in(argv[1], std::ios::binary);
    std::string out = argv[2];
    int k = 0;
    while (true) {
        double t, R[9], p[3]; int32_t n;
        if (!in.read((char*)&t, 8)) break;
        in.read((char*)R, 72); in.read((char*)p, 24); in.read((char*)&n, 4);
        std::vector<float> xyz(3 * (size_t)n);
        in.read((char*)xyz.data(), 12 * (size_t)n);
        boost::shared_ptr<PointCloudXYZI> pc(new PointCloudXYZI());
        pc->resize(n);
        for (int i = 0; i < n; i++) {
            PointType q; q.x = xyz[3*i]; q.y = xyz[3*i+1]; q.z = xyz[3*i+2];
            q.intensity = 0; q.curvature = 0; q.normal_x = q.normal_y = q.normal_z = 0;
            (*pc)[i] = q;
        }
        M3D rot; rot << R[0], R[1], R[2], R[3], R[4], R[5], R[6], R[7], R[8];
        V3D pos(p[0], p[1], p[2]);
        std::stringstream a, b;
        a << out << "/" << std::setw(6) << std::setfill('0') << k << ".label";
        b << out << "/" << std::setw(6) << std::setfill('0') << k << "_o.label";
        F->set_path(a.str(), b.str());
        F->filter(pc, rot, pos, t);
        k++;
    }
    std::cout << "[md_offline] frames " << k << std::endl;
    return 0;
}
