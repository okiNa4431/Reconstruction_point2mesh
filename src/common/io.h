#pragma once
#include <iostream>
#include <fstream>
#include <vector>
#include "point.h"
#include "triangle.h"

enum class fileType
{
    withColor = 0x00,
    nonColor = 0x01,
};

inline void read_txt(const char filename[], std::vector<Point>& points, int& pN, fileType ft) {
    FILE* in = fopen(filename, "r");
    fscanf(in, "%d", &pN);
    points.resize(pN);
    for (int i = 0; i < pN; i++) {
        points[i].index = i;
        if (ft == fileType::withColor) {
            fscanf(in, "%lf %lf %lf %lf %lf %lf %hu %hhu %hhu %hhu", &points[i].x, &points[i].y, &points[i].z, &points[i].normal.x, &points[i].normal.y, &points[i].normal.z, &points[i].CTval, &points[i].r, &points[i].g, &points[i].b);
        }
        else if (ft == fileType::nonColor) {
            fscanf(in, "%lf %lf %lf %lf %lf %lf", &points[i].x, &points[i].y, &points[i].z, &points[i].normal.x, &points[i].normal.y, &points[i].normal.z);
        }
    }
    printf("入力データ読み取り完了\n");
}

inline void write_ply(const std::string& filename, const std::vector<Point>& points, const std::vector<Triangle>& generateTriangles) {
    std::ofstream writer(filename.c_str(), std::ios::out | std::ios::binary);
    if (writer.fail()) {
        throw std::runtime_error("Failed to open file: " + filename);
    }

    //plyファイルの形式通りに記述
    writer << "ply"
        << "\n";
    writer << "format binary_little_endian 1.0"
        << "\n";

    //使っていない点を削除し、その分インデックスを詰める
    std::vector<int> pointNum(points.size(), 0);
    //点のインデックスに対してその点が何個使われているかを数える
    for (int i = 0; i < generateTriangles.size(); i++) {
        pointNum[generateTriangles[i].vertex1.index]++;
        pointNum[generateTriangles[i].vertex2.index]++;
        pointNum[generateTriangles[i].vertex3.index]++;
    }
    //pointNumの存在する点のインデックスの値を新たなインデックスの番号に書き換える
    int countP = 0;
    for (int i = 0; i < points.size(); i++) {
        if (pointNum[i] == 0) {
            pointNum[i] = -1;
            continue;
        }
        pointNum[i] = countP;
        countP++;
    }
    printf("点の数:%d\n", countP);

    writer << "element vertex " << countP << "\n";
    writer << "property float x"
        << "\n";
    writer << "property float y"
        << "\n";
    writer << "property float z"
        << "\n";
    writer << "property float nx"
        << "\n";
    writer << "property float ny"
        << "\n";
    writer << "property float nz"
        << "\n";
    writer << "property uchar red"
        << "\n";
    writer << "property uchar green"
        << "\n";
    writer << "property uchar blue"
        << "\n";
    writer << "property uchar alpha"
        << "\n";
    writer << "property ushort quality"
        << "\n";

    //残りを記述(面回り)
    const int nFaces = (int)generateTriangles.size();
    printf("面の数:%d\n", nFaces);
    writer << "element face " << nFaces << "\n";
    writer << "property list uchar int vertex_indices"
        << "\n";
    writer << "property uchar red"
        << "\n";
    writer << "property uchar green"
        << "\n";
    writer << "property uchar blue"
        << "\n";
    writer << "property uchar alpha"
        << "\n";
    writer << "end_header"
        << "\n";

    for (const auto& p : points) {
        if (pointNum[p.index] == -1) continue;
        float buf_Coordinate[6] = { (float)(p.x), (float)p.y, (float)(p.z), (float)p.normal.x, (float)p.normal.y, (float)p.normal.z };
        unsigned char buf_Color[4] = { p.r, p.g, p.b, 255 };
        unsigned short buf_CT[1] = { p.CTval };
        writer.write((char*)buf_Coordinate, sizeof(float) * 6);
        writer.write((char*)buf_Color, sizeof(unsigned char) * 4);
        writer.write((char*)buf_CT, sizeof(unsigned short) * 1);
    }

    for (int i = 0; i < nFaces; i++) {
        const uint8_t k = 3;
        writer.write((char*)&k, sizeof(uint8_t));
        //a,b,cは三角形を構成する頂点のインデックス
        const int a = pointNum[generateTriangles[i].vertex1.index];
        const int b = pointNum[generateTriangles[i].vertex2.index];
        const int c = pointNum[generateTriangles[i].vertex3.index];
        //面の裏表を調整
        const Point A2B = generateTriangles[i].vertex2 - generateTriangles[i].vertex1;
        const Point A2C = generateTriangles[i].vertex3 - generateTriangles[i].vertex1;
        Array AB = Array(A2B.x, A2B.y, A2B.z);
        Array AC = Array(A2C.x, A2C.y, A2C.z);
        Array faceNormal = AB.cross(AC);
        double angle = faceNormal.calcAngle(generateTriangles[i].vertex1.normal);
        //頂点の法線と面の法線との角度の差が90度を超えるかどうかによって判定
        if (angle < 1.5708) {
            uint32_t buf_Coordinate[3] = { a, b, c };
            writer.write((char*)buf_Coordinate, sizeof(uint32_t) * 3);
        }
        else {
            uint32_t buf_Coordinate[3] = { a, c, b };
            writer.write((char*)buf_Coordinate, sizeof(uint32_t) * 3);
        }

        //色情報を書き込む、面の色は3つの頂点の色の平均とする
        unsigned char red = (points[a].r + points[b].r + points[c].r) / 3;
        unsigned char green = (points[a].g + points[b].g + points[c].g) / 3;
        unsigned char blue = (points[a].b + points[b].b + points[c].b) / 3;
        unsigned char buf_Color[4] = { red, green, blue, 255 };
        writer.write((char*)buf_Color, sizeof(unsigned char) * 4);
    }

    writer.close();
}