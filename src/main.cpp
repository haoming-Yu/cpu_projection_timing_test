#include <iostream>
#include <vector>
#include <chrono>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include "mesh.h"
#include "camera.h"

void project_vertexes_to_cameras(std::vector<float3>& vertexes, Camera::Cam& cam, std::vector<std::vector<float3>>& vertexes_cam) {
    int cam_num = cam.extrinsics_.size();
    for (int i = 0; i < cam_num; i++) {
        vertexes_cam.push_back(std::vector<float3>());
        Eigen::Matrix4f T_cw = cam.extrinsics_[i].T_cw.cast<float>(); // get world to camera extrinsic
        // transform vertexes to camera coordinate
        for (int j = 0; j < vertexes.size(); j++) {
            float3 vertex = vertexes[j];
            Eigen::Vector4f vertex_homo(vertex.x, vertex.y, vertex.z, 1.0);
            Eigen::Vector4f vertex_cam = T_cw * vertex_homo;
            float3 vertex_cam_3d = {vertex_cam(0), vertex_cam(1), vertex_cam(2)};
            if (vertex_cam(2) > 0) {
                // std::cout << "vertex_cam_3d: " << vertex_cam_3d.x << " " << vertex_cam_3d.y << " " << vertex_cam_3d.z << std::endl;
                vertexes_cam[i].push_back(vertex_cam_3d);
                // if (j < 100000) {
                //     std::cout << "vertex " << j << " visible to camera " << i << " " << vertex.x << " " << vertex.y << " " << vertex.z  << std::endl << "camera extrinsic: " << cam.extrinsics_[i].T_cw.matrix() << std::endl;
                // } else {
                //     break;
                // }
            }
        }
    }
}

void project_cameras_to_uv(std::vector<std::vector<float3>>& vertexes_cam, Camera::Cam& cam, std::vector<std::vector<float2>>& uv_cam) {
    int cam_num = cam.extrinsics_.size();
    for (int i = 0; i < cam_num; i++) {
        uv_cam.push_back(std::vector<float2>());
        for (int j = 0; j < vertexes_cam[i].size(); j++) {
            float3 vertex = vertexes_cam[i][j];
            float u = cam.intrinsic_.fx * (vertex.x / vertex.z) + cam.intrinsic_.cx;
            float v = cam.intrinsic_.fy * (vertex.y / vertex.z) + cam.intrinsic_.cy;
            if (u >= 0 && u < cam.intrinsic_.img_width && v >= 0 && v < cam.intrinsic_.img_height) {
                uv_cam[i].push_back(float2{u, v});
                // std::cout << "uv: " << u << " " << v << " visible to camera " << i << std::endl;
            }
        }
        // std::cout << "Vertexes number projected to camera " << i << " (visible to camera " << i << "): " << uv_cam[i].size() << std::endl;
    }
}

int main() {
    MeshProcessing::Mesh mesh;
    mesh.loadFromFile("../data/underground/mesh/filtered_mesh.ply");
    std::vector<float3> vertexes = mesh.getVertexes();
    std::cout << "Vertexes number rechecked: " << vertexes.size() << std::endl;

    Camera::Cam cam;
    cam.loadIntrinsic("../data/underground/intrinsic.log");
    cam.loadExtrinsics("../data/underground/traj.log");
    cam.dump_intrinsic_to_float(); // prepare for the data loading of gpu
    cam.dump_extrinsic_to_float();

    auto start_time = std::chrono::high_resolution_clock::now();
    std::vector<std::vector<float3>> vertexes_cam;
    project_vertexes_to_cameras(vertexes, cam, vertexes_cam);
    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end_time - start_time;
    std::cout << "Time taken to transform vertexes to cameras: " << duration.count() << " seconds" << std::endl;
    // std::cout << "Vertexes number projected to camera 0 (infront of camera 0): " << vertexes_cam[0].size() << std::endl;

    start_time = std::chrono::high_resolution_clock::now();
    std::vector<std::vector<float2>> uv_cam;
    project_cameras_to_uv(vertexes_cam, cam, uv_cam);
    end_time = std::chrono::high_resolution_clock::now();
    duration = end_time - start_time;
    std::cout << "Time taken to project cameras to uv: " << duration.count() << " seconds" << std::endl;
    // std::cout << "Vertexes number projected to camera 0 (infront of camera 0): " << uv_cam[0].size() << std::endl;

    return 0;
}