#pragma once

#include <pangolin/pangolin.h>
#include "Eigen/Geometry"
#include <Eigen/Core>
#include <mutex>

class VisTool {
public:
	VisTool();
	
	~VisTool();
	
	void AddCameraPose(const std::vector<Eigen::Isometry3d> &T_map_cams);
	
	void AddPoints(const std::vector<Eigen::Vector3d> &points);
	
	void Run();

private:
	static inline void DrawLine(const float x1, const float y1, const float z1,
	                     const float x2, const float y2, const float z2) {
		glVertex3f(x1, y1, z1);
		glVertex3f(x2, y2, z2);
	}
	
	void DrawCameraPose();
	
	
	static void DrawAxis(Eigen::Isometry3d &pose);
	
	void DrawPoints();
	
	void DrawHorizontalGrid();
	
	std::string m_window_name{"empty"};
	std::mutex m_lock_Points;
	std::mutex m_lock_Camera_Pose_current;
	std::vector<Eigen::Vector3d> m_points;
	std::deque<Eigen::Isometry3d> m_poses;
	std::vector<Eigen::Isometry3d> m_T_map_current_cams;
};