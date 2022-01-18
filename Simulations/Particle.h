#ifndef PARTICLE_h
#define PARTICLE_h

class Particle
{
public:
	Particle(Vec3 pos, float w, Vec3 c) : pos(pos), tmp_pos(pos), w(w), v(0, 0,0), color(c) {
		p = 0;
		rho = 0;
	}
	
	Vec3 pos;
	Vec3 tmp_pos;
	float w; //Inverse mass 
	Vec3 v; // velocity
	
	float p; // pressure
	float rho; // density
	Vec3 f; // current sum of external forces
	Vec3 color; // color of this particle
	
	float get_mass() {
		if (w == 0.0f) return -1;
		return 1 / w;
	}
};

#endif