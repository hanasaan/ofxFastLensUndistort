#pragma once

#include "ofMain.h"

#define OFX_FAST_LENS_UNDISTORT_BEGIN_NAMESPACE namespace ofx { namespace FastLensUndistort {
#define OFX_FAST_LENS_UNDISTORT_END_NAMESPACE } }

OFX_FAST_LENS_UNDISTORT_BEGIN_NAMESPACE

#define STRINGIFY(x) #x

#define USE_CV // comment out if you do not use opencv

// utility function for converting cv::Mat to float array
#ifdef USE_CV
#include "opencv2/opencv.hpp"
static inline void toFloatArray(const cv::Mat& inputDoubleArray, float* outArray) {
    for (int i=0; i<inputDoubleArray.cols * inputDoubleArray.rows; ++i) {
        outArray[i] = inputDoubleArray.ptr<double>()[i];
    }
}
#endif

static string vertexShader =
STRINGIFY(
    uniform float camera_matrix[9];
    uniform float dist_coeffs[8];
    uniform float undistorted_camera_matrix[9];
    uniform vec2  tex_scale;

    float k1 = dist_coeffs[0];
    float k2 = dist_coeffs[1];
    float p1 = dist_coeffs[2];
    float p2 = dist_coeffs[3];
    float k3 = dist_coeffs[4];
    float k4 = dist_coeffs[5];
    float k5 = dist_coeffs[6];
    float k6 = dist_coeffs[7];

    float cx = camera_matrix[2];
    float cy = camera_matrix[5];
    float fx = camera_matrix[0];
    float fy = camera_matrix[4];
    float ucx = undistorted_camera_matrix[2];
    float ucy = undistorted_camera_matrix[5];
    float ufx = undistorted_camera_matrix[0];
    float ufy = undistorted_camera_matrix[4];
          
    varying vec2 texcoord;

    void main()
    {
        vec2 out_uv = gl_Vertex.xy;

        float x = (out_uv.x - ucx) / ufx;
        float y = (out_uv.y - ucy) / ufy;
        float xy = x * y;
        float x2 = x * x;
        float y2 = y * y;
        float r2 = x2 + y2;
        float r4 = r2 * r2;
        float r6 = r2 * r2 * r2;
        float _2xy = 2.0 * xy;

        float k_radial = (1.0 + k1*r2 + k2*r4 + k3*r6) / (1.0 + k4*r2 + k5*r4 + k6*r6);

        float x_d = x * k_radial + (_2xy * p1 + p2 * (r2 + 2.0 * x2));
        float y_d = y * k_radial + (_2xy * p2 + p1 * (r2 + 2.0 * y2));

        float u = fx * x_d + cx;
        float v = fy * y_d + cy;

        texcoord = vec2(u, v) * tex_scale;
        
        gl_Position = ftransform();
    }
);

static string fragmentShader =
STRINGIFY(
    uniform sampler2DRect tex;
    varying vec2 texcoord;
    void main()
    {
        gl_FragColor = texture2DRect(tex, texcoord);
    }
);

class Undistort
{
private:
    float distorted_camera_matrix[9];
    float dist_coeffs[8];
    float undistorted_camera_matrix[9];
    int width;
    int height;
    
    ofVboMesh mesh;
    ofShader shader;
public:
    void setup(int width, int height, int xDivisionCount = 32)
    {
        this->width = width;
        this->height = height;

        // setup vertices
        int yDivisionCount = max(1, xDivisionCount * height / width);
        
        float stepx = static_cast<float>(width) / xDivisionCount;
        float stepy = static_cast<float>(height) / yDivisionCount;
        
        for (int ys = 0; ys <= yDivisionCount; ys++) {
            float y = stepy * ys;
            for (int xs = 0; xs <= xDivisionCount; xs++) {
                float x = stepx * xs;
                mesh.addVertex(ofVec3f(x, y, 0));
            }
        }
        
        // setup indices
        int vertsx = xDivisionCount + 1;
        int vertsy = yDivisionCount + 1;

        mesh.setMode(OF_PRIMITIVE_TRIANGLE_STRIP);
        for (int yv = 0; yv < yDivisionCount; ++yv) {
            if ((yv & 0x01) == 0) {
                for (int xv = 0; xv < vertsx; ++xv) {
                    int index0 = xv + (yv + 0) * vertsx;
                    int index1 = xv + (yv + 1) * vertsx;
                    mesh.addIndex(index0);
                    mesh.addIndex(index1);
                }
            } else {
                for (int xv = vertsx - 1; xv >= 0; --xv) {
                    int index0 = xv + (yv + 0) * vertsx;
                    int index1 = xv + (yv + 1) * vertsx;
                    mesh.addIndex(index0);
                    mesh.addIndex(index1);
                }
            }
        }
        
        // setup shader
        shader.setupShaderFromSource(GL_VERTEX_SHADER, vertexShader);
        shader.setupShaderFromSource(GL_FRAGMENT_SHADER, fragmentShader);
		shader.linkProgram();
        
        // init
        std::fill(distorted_camera_matrix, distorted_camera_matrix + 9, 0);
        std::fill(dist_coeffs, dist_coeffs + 8, 0);
        std::fill(undistorted_camera_matrix, undistorted_camera_matrix + 9, 0);
    }
    
#ifdef USE_CV
    void setCameraMatrix(const cv::Mat& distortedCameraMatrix, const cv::Mat& distortCoeffs, const cv::Mat& undistortedCameraMatrix = cv::Mat())
    {
        toFloatArray(distortedCameraMatrix, distorted_camera_matrix);
        toFloatArray(distortCoeffs, dist_coeffs);
        
        if (undistortedCameraMatrix.rows) {
            toFloatArray(undistortedCameraMatrix, undistorted_camera_matrix);
        } else {
            toFloatArray(distortedCameraMatrix, undistorted_camera_matrix);
        }
    }
#endif
    
    void setCameraMatrix(float distortedCameraMatrix[9], float distortCoeffs[5], float* undistortedCameraMatrix = NULL)
    {
        std::copy<float*>(distortedCameraMatrix, distortedCameraMatrix + 9, this->distorted_camera_matrix);
		std::copy<float*>(distortCoeffs, distortCoeffs + 5, this->dist_coeffs);
        
        if (undistortedCameraMatrix) {
            std::copy<float*>(undistortedCameraMatrix, undistortedCameraMatrix + 9, this->undistorted_camera_matrix);
        } else {
            std::copy<float*>(distortedCameraMatrix, distortedCameraMatrix + 9, this->undistorted_camera_matrix);
        }
    }
    
    void draw(ofTexture& tex, float  x, float  y, float  w, float  h)
    {
        float sx = w / width;
        float sy = h / height;
        
        ofPushMatrix();
        ofTranslate(x, y);
        ofScale(sx, sy);
        tex.bind();
        shader.begin();
        shader.setUniform2f("tex_scale", tex.getWidth() / width, tex.getHeight() / height);
		shader.setUniform1fv("camera_matrix", distorted_camera_matrix, 9);
		shader.setUniform1fv("undistorted_camera_matrix", undistorted_camera_matrix, 9);
		shader.setUniform1fv("dist_coeffs", dist_coeffs, 8);
        mesh.draw();
		shader.end();
        tex.unbind();
        ofPopMatrix();
    }
};

OFX_FAST_LENS_UNDISTORT_END_NAMESPACE

namespace ofxFastLensUndistort = ofx::FastLensUndistort;
