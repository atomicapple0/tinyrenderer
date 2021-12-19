#include <iostream>
#include <vector>
#include <cmath>
#include "tgaimage.h"
#include "model.h"
#include "geometry.h"

const TGAColor white = TGAColor(255, 255, 255, 255);
const TGAColor red   = TGAColor(255, 0,   0,   255);
const TGAColor green = TGAColor(0, 	 255, 0,   255);
const TGAColor blue  = TGAColor(0, 	 0,   255, 255);

Model *model = NULL;
const int width  = 800;
const int height = 800;

void line(int x0, int y0, int x1, int y1, TGAImage &image, TGAColor color) {
	bool steep;
	if (std::abs(x0-x1)<std::abs(y0-y1)) { // if the line is steep, we transpose the image
		std::swap(x0, y0);
		std::swap(x1, y1);
		steep = true;
	}
	if (x0>x1) { // make it left-to-right
		std::swap(x0, x1);
		std::swap(y0, y1);
	}
	int dx = x1-x0;
	int dy = y1-y0;
	float derror = std::abs(dy/float(dx));
	float error = 0;
	int y = y0;
	for (int x=x0; x<=x1; x++) {
		if (steep) {
			image.set(y, x, color); // if transposed, de−transpose
		} else {
			image.set(x, y, color);
		}
		error += derror;
		if (error>.5) {
			y += (y1>y0?1:-1);
			error -= 1.;
		}
	}
}

void line(Vec2i t0, Vec2i t1, TGAImage &image, TGAColor color) {
	line(t0.x, t0.y, t1.x, t1.y, image, color);
}

Vec3f barycentric(Vec2i *pts, Vec2i P) {
	// S = <up, vp, s>
	Vec3f S =
		Vec3f(pts[2][0] - pts[0][0],
			  pts[1][0] - pts[0][0],
			  pts[0][0] - P[0]) ^
		Vec3f(pts[2][1] - pts[0][1],
			  pts[1][1] - pts[0][1],
			  pts[0][1] - P[1]);
	double up, vp, s, u, v;
	up = S.x; vp = S.y; s = S.z;
	// P = A + u*AB + v*BC AKA
	// P = (1-u-v)*A + u*B + v*C
	u = up / s; v = vp / s;
	// if abs(s) < 1, then s == 0 since s integer
	// thus degenerate case
	if (std::abs(s) < 1) {
		return Vec3f(-1,-1,-1);
	}
	// std::cout << "1.-u-v:" << 1.-u-v << "\n";
	// std::cout << "1.-(up+vp)/s:" << 1.-(up+vp)/s << "\n";
	// std::cout << "up:" << up << "\n";
	// std::cout << "vp:" << vp << "\n";
	// std::cout << "s:" << s << "\n";
	// std::cout << "u:" << u << "\n";
	// std::cout << "v:" << v << "\n";
	// std::cout << "\n";
	// std::cout << "\n";
	// Due to double precision (?): 1.-(up+vp)/s != 1.-u-v
	// The latter results in random black dots in the output for some reason
	return Vec3f(1.-(up+vp)/s, u, v);
}

void triangle(Vec2i *pts, TGAImage &image, TGAColor color) {
	Vec2i bboxmin(image.get_width()-1,image.get_height()-1);
	Vec2i bboxmax(0,0);
	Vec2i clamp(image.get_width()-1,image.get_height()-1);

	for (int i=0; i<3; i++) {
		for (int j=0; j<2; j++) {
			bboxmin[j] = std::max(0, std::min(bboxmin[j], pts[i][j]));
			bboxmax[j] = std::min(clamp[j], std::max(bboxmax[j], pts[i][j]));
		}
	}
	
	// P in ABC iff:
	// 	- u,v,(1-u-v) \in [0,1]
	Vec2i P;
	for (P.x=bboxmin.x; P.x<=bboxmax.x; P.x++) {
		for (P.y=bboxmin.y; P.y<=bboxmax.y; P.y++) {
			Vec3f bc_screen = barycentric(pts, P);
			if (bc_screen.x<0 || bc_screen.y<0 || bc_screen.z<0) continue;
			image.set(P.x, P.y, color);
		}
	}
}

int main(int argc, char** argv) {
	if (2==argc) {
		model = new Model(argv[1]);
	} else {
		model = new Model("obj/african_head.obj");
	}

	Vec3f light_dir = Vec3f(0,0,-1);

	TGAImage image(width, height, TGAImage::RGB);
    for (int i=0; i<model->nfaces(); i++) {
		std::vector<int> face = model->face(i); 
		Vec2i screen_coords[3];
		Vec3f world_coords[3];
		for (int j=0; j<3; j++) {
			Vec3f v = model->vert(face[j]);
			screen_coords[j] = Vec2i((v.x+1.)*width/2., (v.y+1.)*height/2.);
			world_coords[j]  = v;
		}
		Vec3f n = (world_coords[2]-world_coords[0])^(world_coords[1]-world_coords[0]);
		n.normalize();
		float intensity = n*light_dir;
		if (intensity>0) { 
			triangle(screen_coords, image, TGAColor(intensity*255, intensity*255, intensity*255, 255)); 
		} 
	}

	image.flip_vertically(); // i want to have the origin at the left bottom corner of the image
	image.write_tga_file("output.tga");
	return 0;
}

