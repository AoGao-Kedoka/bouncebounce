#pragma once
#include  "Particle.h"
/*class IHasher {
public:
	virtual size_t Hash(const Vec3& pos, size_t length) = 0;
};

class ParticleHasher : public IHasher {
public:
	ParticleHasher(size_t particleSize) : radius{ radius } {};
	virtual size_t Hash(const Vec3& pos, size_t length) override{
		float rx = normalize(pos.x, radius);
		float ry = normalize(pos.y, radius);
		float rz = normalize(pos.z, radius);

		constexpr auto p1 = 73856093;
		constexpr auto p2 = 19349663;
		constexpr auto p3 = 83492791;

		return (
			(rx * p1) ^ (ry * p2) ^ (rz * p3)
			) % length;
	}
private:
	size_t radius;
};

size_t  normalize(float dimension, float radius) {
	return floor(dimension / radius);
}*/