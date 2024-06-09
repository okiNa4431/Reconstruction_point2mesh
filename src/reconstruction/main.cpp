#include "Reconstruction_point2mesh.h"
#include <common/io.h>

int main()
{
	//ファイルから点群のデータを読み取る
	const char input[] = "D:PointCloud/bunny.txt";
	int pN;
	std::vector<Point> points;
	std::vector<Triangle> triangles_T123;//T1~T3三角形
	std::vector<Triangle> triangles_T2;//T2三角形
	read_txt(input, points, pN, fileType::nonColor);

	//kdtreeの構築
	kdtree tree;
	tree.constract(points);

	//ドロネー三角形の抽出
	threePlaneIntersection(points, pN, tree, triangles_T123);
	chooseDisplayedTriangles(triangles_T2, triangles_T123);

	//plyファイル出力
	const char output[] = "D:Mesh/bunny.ply";
	write_ply(output, points, triangles_T2);
}