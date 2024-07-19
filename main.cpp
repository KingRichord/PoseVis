#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sophus/se3.hpp>
#include "Vis.h"
std::shared_ptr<VisTool> vis_tool;

class SplineFusion{
public:
	SplineFusion() = default;
	Eigen::Isometry3d CumulativeForm(const Eigen::Isometry3d& T_1,const Eigen::Isometry3d& T_2,
									 const Eigen::Isometry3d& T_3,const Eigen::Isometry3d& T_4, double u);
	static double GetUt(const double t, const double ti, const double dt){return (t-ti)/dt;};
	static Sophus::SE3d FromAtoB(const Sophus::SE3d& a,const Sophus::SE3d& b);
	static void SE3Eigen2Sophus(Eigen::Isometry3d e,Sophus::SE3d & s);
	void test();
private:
	Sophus::SE3d t1,t2,t3,t4;
};

Eigen::Isometry3d SplineFusion::CumulativeForm(
	const Eigen::Isometry3d& T_1,
	const Eigen::Isometry3d& T_2,
	const Eigen::Isometry3d& T_3,
	const Eigen::Isometry3d& T_4,
	const double u)
{
	Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
	SE3Eigen2Sophus(T_1,t1);
	SE3Eigen2Sophus(T_2,t2);
	SE3Eigen2Sophus(T_3,t3);
	SE3Eigen2Sophus(T_4,t4);
	const Sophus::SE3d cur =   Sophus::SE3d::exp(t1.log())*
			Sophus::SE3d::exp(((5 + 3*u - 3*u*u + u*u*u) / 6.)*FromAtoB(t1,t2).log())*
			Sophus::SE3d::exp(((1 + 3*u + 3*u*u - 2*u*u*u) / 6.)*FromAtoB(t2,t3).log())*
		    Sophus::SE3d::exp(((u*u*u)/6.)*FromAtoB(t3,t4).log());
	result = cur.matrix();
	return result;
	
}

void SplineFusion::SE3Eigen2Sophus(Eigen::Isometry3d e, Sophus::SE3d & s) {
	Eigen::Matrix4d temp = e.matrix();
	const Eigen::Vector3d t(temp(0,3),temp(1,3),temp(2,3));
	const auto t1 = Sophus::SE3d(e.rotation(),t);
	s = t1;
}

Sophus::SE3d SplineFusion::FromAtoB(const Sophus::SE3d& a, const Sophus::SE3d& b) {
	return Sophus::SE3d(a.inverse()*b);
}

void SplineFusion::test() {
	std::vector<Eigen::Isometry3d> poses;
	int num  = 4;
	for (int i = 0; i < num; ++i) {
		Eigen::Isometry3d temp_pose;
		temp_pose.setIdentity();
		temp_pose.rotate(Eigen::AngleAxisd(M_PI/4*(1+i*0.3), Eigen::Vector3d(0,0,1)).toRotationMatrix());
		temp_pose.translate(Eigen::Vector3d(0,i*0.1,i*0.1));
		poses.emplace_back(temp_pose);
	}
	std::vector<Eigen::Vector3d> points;
	for (int j = 3; j < num; ++j) {
		for (double i = 0; i < 10; ++i) {
			Eigen::Isometry3d tf = CumulativeForm(poses[j],poses[j-1],poses[j-2],poses[j-3],i/10.0);
			poses.emplace_back(tf);
			points.emplace_back(tf.translation());
		}
	}
	vis_tool->AddPoints(points);
	vis_tool->AddCameraPose(poses);
}

int main(int argc, char *argv[])
{
	
	vis_tool = std::make_shared<VisTool>();
	auto vis_render = std::thread([] { vis_tool->Run(); });
	SplineFusion sf;
	sf.test();
	while (true) {
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
	return 0;
}
