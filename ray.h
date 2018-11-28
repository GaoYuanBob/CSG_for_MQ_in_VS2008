#pragma once

#include "vec.h"

template <class N>
struct Ray3_CSG
{
	Vec3<N> p; // point of origin
	Vec3<N> r; // ray direction

	Ray3_CSG<N>() {}
	Ray3_CSG<N>(const Vec3<N> &point, const Vec3<N> &dir) :	p(point), r(dir){}
	template<class T>
	Ray3_CSG<N>(const Ray3_CSG<T> &cp) : p(cp.p), r(cp.r) {}
};

template<class N>
inline std::ostream& operator<<(std::ostream &out, const Ray3_CSG<N> &ray) {
	return out << '[' << ray.p << ';' << ray.r << ']';
}

typedef Ray3_CSG<float> Ray3f;
typedef Ray3_CSG<double> Ray3d_CSG;

