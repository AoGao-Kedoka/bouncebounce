#include "SpatialHash.h"
/*
SpatialHash::SpatialHash(const std::vector<Particle&>& particles, float h) {
    this->h = h;
	this->hasher = new ParticleHasher{this->h};

	this->hashTable.resize(this->resize_to_prime(2 * particles.size()));

	for (const auto& particle : particles) {
		this->[particle].insert(particle);
	}
}

std::list<Particle&> SpatialHash::collisions(const Particle& p) {
    float bbmin = normalize(p.pos  - Vec3(this->h, this->h, this->h));
    float bbmax = normalize(p.pos + Vec3(this->h, this->h, this->h));

    std::list<const Particle&> collisions;

    for (auto x = bbmin; x < bbmax; x++) {
        for (auto y = bbmin; y < bbmax; y++) {
            for (auto z = bbmin; z < bbmax; z++) {
                for (const auto& particle : this->[Vec3(x, y, z)]) {
                    if (abs(particle-p) <= h) {
                        collisions.insert(particle);
                    }
                }
            }
        }
    }

    return collisions;
}

std::list<Particle&> SpatialHash::operator[](const Vec3& pos) {
    return this->hashTable[this->hasher->Hash(pos, this->hashTable.size())];
}

void SpatialHash::recalculate() {
    for (int i = 0; i < hashTable.size(); i++) {
        this->hashTable[i].insert(particle);
    }
}

size_t SpatialHash::resize_to_prime(size_t minimum) { //!check valid
    while (true) {
        minimum++;

        if (minimum == 2 || minimum == 3)
            continue;

        if (minimum % 2 == 0 || minimum % 3 == 0)
            continue;

        int divisor = 6;
        while (divisor * divisor - 2 * divisor + 1 <= minimum) {

            if (     minimum % (divisor - 1) == 0
                || minimum % (divisor + 1) == 0)
                continue;

            divisor += 6;

        }

        return minimum;

    }
}
*/