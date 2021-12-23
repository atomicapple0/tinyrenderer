#include <iostream>
#include <vector>
#include <cmath>
#include <limits>
#include "tgaimage.h"
#include "model.h"
#include "geometry.h"

const TGAColor white = TGAColor(255, 255, 255, 255);
const TGAColor red   = TGAColor(255, 0,   0,   255);
const TGAColor green = TGAColor(0, 	 255, 0,   255);
const TGAColor blue  = TGAColor(0, 	 0,   255, 255);

const int width  = 800;
const int height = 800;
const int depth  = 255;

Model *model = new Model("obj/african_head.obj");
float *zbuffer = new float[width*height];
Vec3f light_dir(0,0,-1);
Vec3f camera(0,0,3);

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

Vec3f barycentric(Vec3f pts[3], Vec3f P) {
	// S = <up, vp, s>
	Vec3f S =
		Vec3f(pts[2].x - pts[0].x,
			  pts[1].x - pts[0].x,
			  pts[0].x - P.x) ^
		Vec3f(pts[2].y - pts[0].y,
			  pts[1].y - pts[0].y,
			  pts[0].y - P.y);
	double up, vp, s, u, v;
	up = S.x; vp = S.y; s = S.z;
	// P = A + u*AB + v*BC AKA
	// P = (1-u-v)*A + u*B + v*C
	u = up / s; v = vp / s;
	if (std::abs(s) > .01) {
		// due to floating point precision: 1.-(up+vp)/s != 1.-u-v
		return Vec3f(1.-u-v, v, u);
	}
	// degenerate case
	return Vec3f(-1,-1,-1);
}

void triangle(Vec3f pts[3], Vec2f uvs[3], TGAImage &image, float *zbuffer) {
    Vec2f bboxmin( std::numeric_limits<float>::max(),  std::numeric_limits<float>::max());
    Vec2f bboxmax(-std::numeric_limits<float>::max(), -std::numeric_limits<float>::max());
	Vec2f clamp(image.get_width()-1,image.get_height()-1);
	for (int i=0; i<3; i++) {
		for (int j=0; j<2; j++) {
			bboxmin[j] = std::max(0.f, 		std::min(bboxmin[j], pts[i][j]));
			bboxmax[j] = std::min(clamp[j], std::max(bboxmax[j], pts[i][j]));
		}
	}
	// P in ABC iff u,v,(1-u-v) \in [0,1]
	Vec3f P;
	for (P.x=bboxmin.x; P.x<=bboxmax.x; P.x++) {
		for (P.y=bboxmin.y; P.y<=bboxmax.y; P.y++) {
			Vec3f bc_screen = barycentric(pts, P);
			// leniancy for floating point error
			float err = -.001;
			if (bc_screen.x<err || bc_screen.y<err || bc_screen.z<err) continue;
			P.z = 0;
			Vec2f uvP(0,0);
			for (int i=0; i<3; i++) {
				P.z += pts[i].z*bc_screen[i];
				uvP.x += uvs[i].x*bc_screen[i];
				uvP.y += uvs[i].y*bc_screen[i];
			}
			if (zbuffer[int(P.x+P.y*width)]<P.z) {
				zbuffer[int(P.x+P.y*width)] = P.z;
				// TGAColor color = model->diffuse(uvs[0].x,uvs[0].y);
				TGAColor color = model->diffuse(uvP);
				image.set(P.x, P.y, color);
			}
		}
	}
}

Vec3f world2screen(Vec3f v) {
    return Vec3f(int((v.x+1.)*width/2.+.5), int((v.y+1.)*height/2.+.5), v.z);
}

Vec3f screen2world(Vec3f v) {
	return Vec3f(int((v.x-.5)*2./width-1.), int((v.y-.5)*2./height-1.), v.z);
}


int main(int argc, char** argv) {
	for (int i=0; i<width*height; i++) {
		zbuffer[i] = -std::numeric_limits<float>::max();
	}

	TGAImage image(width, height, TGAImage::RGB);
	for (int i=0; i<model->nfaces(); i++) {
        std::vector<int> face = model->face(i);
        Vec3f screen_coords[3];
        Vec3f world_coords[3];
        Vec2f texture_coords[3];
        for (int j=0; j<3; j++) {
			world_coords[j] = model->vert(face[j]);
			screen_coords[j] = world2screen(model->vert(face[j]));
			texture_coords[j] = model->uv(i,j); 
		}
		// Vec3f n = (world_coords[2]-world_coords[0])^(world_coords[1]-world_coords[0]);
		// n.normalize();
		// float intensity = n*light_dir;
		// TGAColor color = TGAColor(intensity*255, intensity*255, intensity*255, 255);
        triangle(screen_coords, texture_coords, image, zbuffer);
    }

	image.flip_vertically(); // i want to have the origin at the left bottom corner of the image
	image.write_tga_file("output.tga");
	return 0;
}