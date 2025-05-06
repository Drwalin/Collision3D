// This file is part of Collision3D.
// Copyright (c) 2025 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#pragma once

#include <vector>

namespace Collision3D
{
template <typename T> struct Matrix {
	Matrix(Matrix &&other) = default;
	Matrix(Matrix &other) = default;
	Matrix(const Matrix &other) = default;
	Matrix &operator=(Matrix &&other) = default;
	Matrix &operator=(Matrix &other) = default;
	Matrix &operator=(const Matrix &other) = default;
	Matrix() : width(-1), height(-1) {}
	
	Matrix(int width, int height);

	T &operator[](int x, int y) { return heights[y * height + x]; }
	T operator[](int x, int y) const { return heights[y * height + x]; }

	int width, height;
	std::vector<T> heights;
};
} // namespace Collision3D
