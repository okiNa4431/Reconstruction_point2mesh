#include "Reconstruction_point2mesh.h"
#include "Eigen/Core"
#include "Eigen/LU"

//注目している点を含めて31点
const int nearestNum = 31;

const double epsilon = 1e-20;
double radius = 1.0;
//平面の係数を計算するクラス
class ComputeCofficient {
public:
	//3点を通る平面の係数の計算
	void planeCofficientCompute(const Point S, const Point T, const Point U, std::vector<double>& cofficients) {
		Array ST, SU;
		ST.x = T.x - S.x;
		ST.y = T.y - S.y;
		ST.z = T.z - S.z;
		SU.x = U.x - S.x;
		SU.y = U.y - S.y;
		SU.z = U.z - S.z;

		cofficients.push_back(ST.y * SU.z - SU.y * ST.z);
		cofficients.push_back(ST.z * SU.x - SU.z * ST.x);
		cofficients.push_back(ST.x * SU.y - SU.x * ST.y);
		cofficients.push_back(-(cofficients[0] * S.x + cofficients[1] * S.y + cofficients[2] * S.z));
	}
	//垂直二等分面の係数の計算
	void planeCofficientCompute(const Point S, const Point T, std::vector<double>& cofficients) {
		//中点
		Point midPoint = S + T;
		midPoint = midPoint / 2;

		//二等分面の法線
		Array normal;
		normal = S.differentialVector(T);
		normal.normalize();

		//係数の代入
		cofficients.push_back(normal[0]);
		cofficients.push_back(normal[1]);
		cofficients.push_back(normal[2]);
		double d = 0.0;
		for (int i = 0; i < 3; i++) {
			d -= normal[i] * midPoint[i];
		}
		cofficients.push_back(d);
	}
	//一つの点の持つ法線と座標を持つ平面の係数の計算
	void planeCofficientCompute(const Point S, std::vector<double>& cofficients) {
		//係数の代入
		cofficients.push_back(S.normal[0]);
		cofficients.push_back(S.normal[1]);
		cofficients.push_back(S.normal[2]);
		double d = 0.0;
		for (int i = 0; i < 3; i++) {
			d -= S.normal[i] * S[i];
		}
		cofficients.push_back(d);
	}
};

ComputeCofficient computeCofficient;

void threePlaneIntersection(const std::vector<Point>& points, const int pN, kdtree& tree, std::vector<Triangle>& triangles) {
	printf("computeDT(3平面の交点ver)開始\n");
	const int pN1000 = pN / 1000;
	for (int i = 0; i < pN; i++) {
		//i番目の点のn近傍の点を取得する
		std::vector<Point> nearestPoints = tree.nearestsbyBall(points[i], nearestNum, radius);
		//std::vector<Point> nearestPoints = tree.nearests(points[i], nearestNum);
		if (i % pN1000 == 0) {
			printf("\r\033[0K");
			printf("ComputeDT %d/1000", (int)i / pN1000);
		}

		//sに注目点のインデックスを追加
		int s = -1;
		for (int j = 0; j < nearestNum; j++) {
			if (nearestPoints[j].x == points[i].x && nearestPoints[j].y == points[i].y && nearestPoints[j].z == points[i].z) {
				s = j;
			}
		}

		//近傍点を全探索して条件を満たすか確認していく
		for (int t = 0; t < nearestNum; t++) {
			for (int u = t + 1; u < nearestNum; u++) {
				if (s == t || s == u) continue;
				//sの持つ法線を保持する平面と、sとtの垂直二等分面と、sとuの垂直二等分面の交点を調べる
					//行列とベクトルを定義して係数を入れていく
				Eigen::Matrix3d A;
				Eigen::Vector3d b;
				std::vector<double> cofficients;

				//sの持つ法線を保持する平面
				computeCofficient.planeCofficientCompute(nearestPoints[s], cofficients);
				A(0, 0) = cofficients[0], A(0, 1) = cofficients[1], A(0, 2) = cofficients[2];
				b(0) = -cofficients[3];
				//sとtの垂直二等分面
				cofficients.resize(0);
				computeCofficient.planeCofficientCompute(nearestPoints[s], nearestPoints[t], cofficients);
				A(1, 0) = cofficients[0], A(1, 1) = cofficients[1], A(1, 2) = cofficients[2];
				b(1) = -cofficients[3];
				//sとuの垂直二等分面
				cofficients.resize(0);
				computeCofficient.planeCofficientCompute(nearestPoints[s], nearestPoints[u], cofficients);
				A(2, 0) = cofficients[0], A(2, 1) = cofficients[1], A(2, 2) = cofficients[2];
				b(2) = -cofficients[3];

				Eigen::Vector3d x = A.partialPivLu().solve(b);
				Point q;
				q.x = x(0); q.y = x(1); q.z = x(2);

				//点t,uを除く近傍点mと点qが、sとmの垂直二等分面の同じ側に存在することを確認していく
				for (int m = 0; m < nearestNum; m++) {
					if (m == s || m == t || m == u) continue;
					cofficients.resize(0);
					//sとmの垂直二等分面を計算
					computeCofficient.planeCofficientCompute(nearestPoints[s], nearestPoints[m], cofficients);
					double sSide = 0, qSide = 0;
					for (int n = 0; n < 3; n++) {
						sSide += cofficients[n] * nearestPoints[s][n];
						qSide += cofficients[n] * q[n];
					}
					sSide += cofficients[3];
					qSide += cofficients[3];

					//同じ側にあるかどうかを確認。ない場合はbreak
					if (sSide * qSide < 0) break;
					//全ての近傍点が条件を満たしていたら小さい方から順に点のインデックスを格納する
					if (m == nearestNum - 1) {
						triangles.push_back(Triangle(nearestPoints[s], nearestPoints[t], nearestPoints[u]));
					}
				}
			}
		}
	}
	printf("\n");
	//ソート
	std::sort(triangles.begin(), triangles.end());}

//どの三角形を表示するかを決める関数
void chooseDisplayedTriangles(std::vector<Triangle>& generateTriangles, const std::vector<Triangle>& indices) {
	Triangle previousTriangle;
	int triN = 1;
	for (int i = 0; i < indices.size(); i++) {
		if (indices[i] == previousTriangle) {
			triN++;
		}
		else {
			triN = 1;
		}
		//三角形が3連続で揃ったら
		if (triN == 2) {
			triN = 0;
			generateTriangles.push_back(indices[i]);
		}
		previousTriangle = indices[i];
	}
}