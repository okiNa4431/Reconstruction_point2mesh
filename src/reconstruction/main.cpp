#include "Reconstruction_point2mesh.h"
#include <common/io.h>

int main()
{
	//�t�@�C������_�Q�̃f�[�^��ǂݎ��
	const char input[] = "D:PointCloud/bunny.txt";
	int pN;
	std::vector<Point> points;
	std::vector<Triangle> triangles_T123;//T1~T3�O�p�`
	std::vector<Triangle> triangles_T2;//T2�O�p�`
	read_txt(input, points, pN, fileType::nonColor);

	//kdtree�̍\�z
	kdtree tree;
	tree.constract(points);

	//�h���l�[�O�p�`�̒��o
	threePlaneIntersection(points, pN, tree, triangles_T123);
	chooseDisplayedTriangles(triangles_T2, triangles_T123);

	//ply�t�@�C���o��
	const char output[] = "D:Mesh/bunny.ply";
	write_ply(output, points, triangles_T2);
}