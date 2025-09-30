// Copyright (c) 2021 Mathema GmbH
// SPDX-License-Identifier: BSD-3-Clause
// Author: Günter Woigk (Kio!)
// Copyright (c) 2021 kio@little-bat.de
// BSD 2-clause license

#pragma once

#include <cmath>
#include <cstdint>

template<typename T> static inline T min(T a,T b) { return a<=b?a:b; }
template<typename T> static inline T max(T a,T b) { return a>=b?a:b; }

template<typename T>
struct TDist
{
	T dx = 0, dy = 0;
	TDist(){}
	TDist (T dx, T dy) : dx(dx), dy(dy) {}
	T length() const noexcept { return sqrt(dx*dx+dy*dy); }
	TDist normalized () { return TDist(*this) / length(); }
	TDist& operator+= (const TDist& q) { dx+=q.dx; dy+=q.dy; return *this; }
	TDist operator+ (TDist q) const { return q += *this; }
	TDist& operator-= (const TDist& q) { dx-=q.dx; dy-=q.dy; return *this; }
	TDist operator- (const TDist& q) const { return TDist{dx-q.dx,dy-q.dy}; }
	TDist& operator*= (T f) { dx*=f; dy*=f; return *this; }
	TDist operator* (T f) const { return TDist{dx*f,dy*f}; }
	TDist& operator/= (T f) { dx/=f; dy/=f; return *this; }
	TDist operator/ (T d) const { return TDist{dx/d,dy/d}; }
	friend bool operator== (const TDist& lhs, const TDist& rhs) { return lhs.dx == rhs.dx && lhs.dy == rhs.dy; }
	TDist& rotate(T rad) {
		const T s = sin(rad); const T c = cos(rad);
		const T x = c * dx - s * dy; const T y = c * dy + s * dx;
		dx = x; dy = y; return *this;
	}
};

template<typename T>
struct TPoint
{
	T x = 0, y = 0;
	TPoint(){}
	TPoint(T x, T y) : x(x), y(y) {}
	TPoint operator+ (const TDist<T> &d) const { return TPoint(x + d.dx, y + d.dy); }
	TPoint& operator+= (const TDist<T> &d) { x += d.dx; y += d.dy; return *this; }
	TPoint operator- (const TDist<T>& d) const { return TPoint(x - d.dx, y - d.dy); }
	TDist<T> operator- (const TPoint &d) const { return TDist<T>(x - d.x, y - d.y); }
	bool operator!= (const TPoint& q) const noexcept { return x != q.x || y != q.y; }
    friend bool operator== (const TPoint& lhs, const TPoint& rhs) { return lhs.x == rhs.x && lhs.y == rhs.y; }
};

template<typename T>
struct TRect
{
	T top = 0, left = 0, bottom = 0, right = 0;
	TRect(){}
	TRect(T top, T left, T bottom, T right) : top(top), left(left), bottom(bottom), right(right) {}
	TRect(const TPoint<T> &topleft, const TPoint<T> &bottomright) : top(topleft.y), left(topleft.x), bottom(bottomright.y), right(bottomright.x) {}
	T width() const noexcept { return right - left; }
	T height() const noexcept { return top - bottom; }
	TPoint<T> bottom_left() const noexcept { return TPoint<T>{left,bottom}; }
	TPoint<T> bottom_right() const noexcept { return TPoint<T>{right,bottom}; }
	TPoint<T> top_left() const noexcept { return TPoint<T>{left,top}; }
	TPoint<T> top_right() const noexcept { return TPoint<T>{right,top}; }
	TPoint<T> center() const noexcept { return TPoint<T>{(left+right)/2,(bottom+top)/2}; }
};

template<typename T>
struct TTransformation
{
	T fx=1, fy=1, sx=0, sy=0, dx=0, dy=0;
	T px=0, py=0, pz=1;
	bool is_projected = false;
	TTransformation()=default;
	TTransformation(T fx,T fy,T sx,T sy, T dx, T dy) : fx(fx),fy(fy),sx(sx),sy(sy),dx(dx),dy(dy){}
	void transform (TPoint<T>& p) {
		T x = p.x; T y = p.y;
		p.x = fx*x + sx*y + dx;
		p.y = fy*y + sy*x + dy;
		if (is_projected) {
			T q = px*x + py*y + pz;
			p.x /= q; p.y /= q;
		}
	}
    TTransformation& addTransformation(const TTransformation& t) {
        T dx1=t.dx, dy1=t.dy, fx1=t.fx, fy1=t.fy, sx1=t.sx, sy1=t.sy;
		const T dx2=dx, dy2=dy, fx2=fx, fy2=fy, sx2=sx, sy2=sy;
		dx = dx2 + dx1*fx2 + dy1*sx2;
		dy = dy2 + dx1*sy2 + dy1*fy2;
		fx = fx1*fx2 + sy1*sx2;
		sy = fx1*sy2 + sy1*fy2;
		sx = sx1*fx2 + fy1*sx2;
		fy = sx1*sy2 + fy1*fy2;
        return *this;
    }
	TTransformation& rotate (T rad) {
        TTransformation rot(cos(rad), cos(rad), -sin(rad), sin(rad), 0, 0);
        *this = TTransformation(*this).addTransformation(rot);
		return *this;
	}
    TTransformation& scale (T s) {
        TTransformation sc(s,s,0,0,0,0);
        *this = TTransformation(*this).addTransformation(sc);
        return *this;
    }
    TTransformation& scale (T x, T y) {
        TTransformation sc(x,y,0,0,0,0);
        *this = TTransformation(*this).addTransformation(sc);
        return *this;
    }
    TTransformation& addOffset (T x, T y) {
        dx += x; dy += y;
        return *this;
    }
	TTransformation& reset() { new(this) TTransformation(); return *this; }
};

using Point = TPoint<float>;
using Dist = TDist<float>;
using Rect = TRect<float>;
using Transformation = TTransformation<float>;
