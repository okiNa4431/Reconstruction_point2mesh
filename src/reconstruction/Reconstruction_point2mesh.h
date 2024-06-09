#pragma once
#include <vector>
#include <common/point.h>
#include <common/array.h>
#include <common/triangle.h>
#include <common/kdtree.h>

using namespace std;


void threePlaneIntersection(const std::vector<Point>& points, const int pN, kdtree& tree, std::vector<Triangle>& triangles);
void chooseDisplayedTriangles(std::vector<Triangle>& generateTriangles, const std::vector<Triangle>& indices);