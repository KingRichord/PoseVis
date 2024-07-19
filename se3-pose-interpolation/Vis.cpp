#include "Vis.h"

VisTool::VisTool() {
	pangolin::CreateWindowAndBind(m_window_name, 1024, 768);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	pangolin::GetBoundWindow()->RemoveCurrent();
}

VisTool::~VisTool() {
	pangolin::FinishFrame();
	pangolin::GetBoundWindow()->RemoveCurrent();
}

void VisTool::Run() {
	pangolin::BindToContext(m_window_name);
	pangolin::OpenGlRenderState s_cam(
			pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.0001, 1000),
			pangolin::ModelViewLookAt(-10, 0, 10, 0, 0, 0, pangolin::AxisZ));
	pangolin::Handler3D handler(s_cam);
	pangolin::View &d_cam = pangolin::CreateDisplay()
			.SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0f / 768.0f)
			.SetHandler(&handler);
	
	while (!pangolin::ShouldQuit()) {
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		d_cam.Activate(s_cam);
		glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
		DrawHorizontalGrid();
		DrawCameraPose();
		DrawPoints();
		glFlush();
		pangolin::FinishFrame();
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
	}
	pangolin::GetBoundWindow()->RemoveCurrent();
}

void VisTool::AddCameraPose(const std::vector<Eigen::Isometry3d> &T_map_cams)  {
	std::lock_guard<std::mutex> lg(m_lock_Camera_Pose_current);
	m_T_map_current_cams.clear();
	m_T_map_current_cams.shrink_to_fit();
	m_T_map_current_cams = T_map_cams;
}


void VisTool::AddPoints(const std::vector<Eigen::Vector3d> &points) {
	std::lock_guard<std::mutex> lg(m_lock_Points);
	m_points.clear();
	m_points = points;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void VisTool::DrawAxis(Eigen::Isometry3d &pose) {
	Eigen::Vector3d Ow = pose.translation();
	Eigen::Vector3d Xw = pose * (1 * Eigen::Vector3d(1, 0, 0));
	Eigen::Vector3d Yw = pose * (1 * Eigen::Vector3d(0, 1, 0));
	Eigen::Vector3d Zw = pose * (1 * Eigen::Vector3d(0, 0, 1));
	glLineWidth(1);
	glBegin(GL_LINES);
	glColor3f(1.0, 0.0, 0.0);
	glVertex3d(Ow[0], Ow[1], Ow[2]);
	glVertex3d(Xw[0], Xw[1], Xw[2]);
	glColor3f(0.0, 1.0, 0.0);
	glVertex3d(Ow[0], Ow[1], Ow[2]);
	glVertex3d(Yw[0], Yw[1], Yw[2]);
	glColor3f(0.0, 0.0, 1.0);
	glVertex3d(Ow[0], Ow[1], Ow[2]);
	glVertex3d(Zw[0], Zw[1], Zw[2]);
	glEnd();
}

void VisTool::DrawCameraPose() {
	m_lock_Camera_Pose_current.lock();
	for (auto &m_T_map_current_cam: m_T_map_current_cams) {
		DrawAxis(m_T_map_current_cam);
	}
	m_lock_Camera_Pose_current.unlock();
}

void VisTool::DrawPoints() {
	m_lock_Points.lock();
	glPointSize(10);
	if (!m_points.empty())
		for (auto const &pt: m_points) {
			glBegin(GL_POINTS);
			glColor3f(1., 0., 0.);
			glVertex3f((GLfloat)pt.x(), (GLfloat)pt.y(), (GLfloat)pt.z());
			glEnd();
		}
	m_lock_Points.unlock();
	
}

void VisTool::DrawHorizontalGrid() {
	
	Eigen::Matrix4f origin;
	origin << 1, 0, 0, 0,
		  0, 1, 0, 0,
		  0, 0, 1, 0,
		  0, 0, 0, 1;
	glPushMatrix();
	glMultTransposeMatrixf(origin.data());
	
	glLineWidth(1);
	std::array<float, 3> grid{};
	grid = {{0.7f, 0.7f, 0.7f}};
	glColor3fv(grid.data());
	
	glBegin(GL_LINES);
	
	constexpr float kIntervalRatio = 0.1;
	constexpr float kGridMin = -100.0f * kIntervalRatio;
	constexpr float kGridMax = 100.0f * kIntervalRatio;
	
	for (int x = -10; x <= 10; x += 1) {
		DrawLine((float)x * 10.0f * kIntervalRatio, kGridMin, 0, x * 10.0f * kIntervalRatio, kGridMax, 0);
	}
	for (int y = -10; y <= 10; y += 1) {
		DrawLine(kGridMin, (float)y * 10.0f * kIntervalRatio, 0, kGridMax, y * 10.0f * kIntervalRatio, 0);
	}
	glEnd();
	glPopMatrix();
}

