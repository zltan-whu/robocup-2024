#ifndef YOLOV5_
#define YOLOV5_

#include <chrono>
#include <cmath>
#include <iostream>
#include <map>
#include <vector>
#include <algorithm>
#include "basic_transform.h"
#include "common.h"
#include "cuda_utils.h"
#include "logging.h"
#include "utils.h"
#include "yololayer.h"

#define USE_FP16 // set USE_INT8 or USE_FP16 or USE_FP32
#define DEVICE 0 // GPU id
#define NMS_THRESH 0.4
#define CONF_THRESH 0.5
#define BATCH_SIZE 1
static const int INPUT_H = Yolo::INPUT_H;
static const int INPUT_W = Yolo::INPUT_W;
static const int OUTPUT_SIZE =
    Yolo::MAX_OUTPUT_BBOX_COUNT * sizeof(Yolo::Detection) / sizeof(float) +
    1; // we assume the yololayer outputs no more than MAX_OUTPUT_BBOX_COUNT
       // boxes that conf >= 0.1
static const char *INPUT_BLOB_NAME = "data";
static Logger_trt gLogger;


using namespace std;
using namespace cv;

// 相机内参矩阵
const Mat camera_matrix = (Mat_<double>(3, 3) << 1360.008221, 0, 978.782472,0, 1357.437959, 583.636339,0.00, 0.0, 1.0);
// 相机畸变系数
const Mat dist_coeffs = (Mat_<double>(5, 1) << -0.309833, 0.043774,-0.001171, -0.001053, 0.000000);
// 3D 特征点世界坐标，与像素坐标对应，单位是mm




class yolov5
{
public:
    yolov5(std::string engine_name)
    {
        this->engine_name = engine_name;
        std::cout<<"this->engine_name: "<<this->engine_name<<std::endl;
        std::ifstream file(engine_name, std::ios::binary);
        size_t size = 0;
        file.seekg(0, file.end);
        size = file.tellg();
        file.seekg(0, file.beg);
        this->trtModelStream = new char[size];
        assert(this->trtModelStream);
        file.read(this->trtModelStream, size);
        file.close();
        std::vector<std::string> file_names;

        this->runtime = createInferRuntime(gLogger);
        assert(runtime != nullptr);
        this->engine = runtime->deserializeCudaEngine(trtModelStream, size);
        assert(engine != nullptr);
        this->context = engine->createExecutionContext();
        assert(context != nullptr);
        this->m_InputBindingIndex = engine->getBindingIndex(INPUT_BLOB_NAME);
        delete[] trtModelStream;

        CUDA_CHECK(cudaMalloc(&buffers[0], 3 * INPUT_H * INPUT_W * sizeof(float)));
        CUDA_CHECK(cudaMalloc(&buffers[1], OUTPUT_SIZE * sizeof(float)));

        CUDA_CHECK(cudaMalloc((void **)&d_resized_img, 3 * INPUT_H * INPUT_H * sizeof(unsigned char)));
        CUDA_CHECK(cudaMalloc((void **)&d_norm_img, sizeof(float) * INPUT_H * INPUT_H * 3));
    }
    ~yolov5()
    {
        context->destroy();
        engine->destroy();
        runtime->destroy();

        CUDA_CHECK(cudaFree(buffers[0]));
        CUDA_CHECK(cudaFree(buffers[1]));
    }
    std::map<int,std::vector<double>> detect(unsigned char *d_roi_image, int roi_w, int roi_h,cv::Mat &img,std::vector<int> detect_type)//cuda的一种图片、宽、高、原始图片
    {
        float image_ratio = roi_w > roi_h ? float(INPUT_W) / float(roi_w) : float(INPUT_H) / float(roi_h);//缩放比例 缩放到640 640
        int width_out = roi_w > roi_h ? INPUT_W : (int)(roi_w * image_ratio);
        int height_out = roi_w < roi_h ? INPUT_H : (int)(roi_h * image_ratio);
        cudaMemset(d_resized_img, 0, sizeof(unsigned char) * INPUT_H * INPUT_W * 3);
        //前处理
        PicResize(d_roi_image, d_resized_img, roi_w, roi_h, width_out,
                   height_out);
        PicNormalize(d_resized_img, d_norm_img, INPUT_W, INPUT_H);

        cudaStream_t stream;
        CUDA_CHECK(cudaStreamCreate(&stream));
        buffers[0] = d_norm_img;
        this->context->enqueue(1, this->buffers, stream, nullptr);
        CUDA_CHECK(cudaMemcpyAsync(this->out_put, this->buffers[1], OUTPUT_SIZE * sizeof(float), cudaMemcpyDeviceToHost, stream));
        cudaStreamSynchronize(stream);
        cudaStreamDestroy(stream);
        std::vector<std::vector<Yolo::Detection>> batch_res(1);
        std::vector<Yolo::Detection> &res = batch_res[0];
        nms(res, &out_put[0], CONF_THRESH, NMS_THRESH);
        // 旋转向量
        Mat rotation_vector;
        // 平移向量
        Mat translation_vector = cv::Mat::zeros(3,1,CV_32F);
        Mat t;
        translation_vector.at<float>(0,0) = 0.0f;
        translation_vector.at<float>(1,0) = 0.0f;
        translation_vector.at<float>(2,0) = 0.0f;
        std::map<int,std::vector<double>> type_dis;//种类和距离
        // cout<<"target detect:"<<endl;
        for (size_t j = 0; j < res.size(); j++)
        {   //只有检测种类会被打印出来并做pnp
            cout<<res[j].class_id<<endl;
            //auto it = std::find(detect_type.begin(),detect_type.end(),(int)res[j].class_id);
            //if(it != detect_type.end()){//说明检测出的种类在待检测的列表中
            if(res[j].conf > 0.7){
                cv::Rect r = get_rect(roi_w, roi_h, res[j].bbox);
                double ratio = (double)r.width/(double)r.height;
                // if(ratio > 1.1 || ratio < 0.9) continue;
                // else{//正方形才会被检测
                cv::rectangle(img, r, cv::Scalar(0x27, 0xC1, 0x36), 4);
                cv::putText(img, std::to_string((int)res[j].class_id), cv::Point(r.x, r.y + 10), cv::FONT_HERSHEY_PLAIN, 3, cv::Scalar(0,0, 255), 2);
                //打印置信度
                cv::putText(img, std::to_string(res[j].conf), cv::Point(r.x, r.y - 10), cv::FONT_HERSHEY_PLAIN, 3, cv::Scalar(0, 0, 255), 4);
                //从左上角开始顺时针
                std::vector<Point2d> image_points;
                image_points.push_back(Point2d(r.x, r.y));//左上角
                image_points.push_back(Point2d(r.x+r.width, r.y));
                image_points.push_back(Point2d(r.x+r.width, r.y+r.height));
                image_points.push_back(Point2d(r.x, r.y+r.height));
                // 3D 特征点世界坐标，与像素坐标对应，单位是cm
                std::vector<Point3d> model_points;
                model_points.push_back(Point3d(-29.9, -29.9, 0.0)); // 左上角 单位 cm
                model_points.push_back(Point3d(+29.9, -29.9, 0.0));
                model_points.push_back(Point3d(+29.9, +29.9, 0.0));
                model_points.push_back(Point3d(-29.9, +29.9, 0.0));
                
                // pnp求解
                //solvePnP(model_points, image_points, camera_matrix, dist_coeffs, rotation_vector, translation_vector);
                solvePnP(model_points, image_points, camera_matrix, dist_coeffs, rotation_vector, t);
                //cout << "Translation Vector:" << endl << translation_vector << endl;
                // float distance = translation_vector.at<float>(0,0)*translation_vector.at<float>(0,0)+translation_vector.at<float>(1,0)*translation_vector.at<float>(1,0)+
                //                   translation_vector.at<float>(2,0)*translation_vector.at<float>(2,0);
                type_dis[(int)res[j].class_id] = {t.at<double>(0,0),t.at<double>(1,0),t.at<double>(2,0)};
                //cout<<"type_dis:"<<translation_vector.at<float>(0,0)<<" "<<translation_vector.at<float>(1,0)<<" "<<translation_vector.at<float>(2,0)<<endl;
                //cout<<"type_dis_double:"<<t.at<double>(0,0)<<" "<<t.at<double>(1,0)<<" "<<t.at<double>(2,0)<<endl;
                //}
            } 
            // cv::Rect r = get_rect(roi_w, roi_h, res[j].bbox);
            // cv::rectangle(img, r, cv::Scalar(0x27, 0xC1, 0x36), 1);
            // cv::putText(img, std::to_string((int)res[j].class_id), cv::Point(r.x, r.y + 5), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0xFF, 0xFF, 0xFF), 2);
            // //打印置信度
            // cv::putText(img, std::to_string(res[j].conf), cv::Point(r.x, r.y - 5), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0xFF, 0xFF, 0xFF), 2);
        }
        return type_dis;
    };
    void undisort(cv::Mat& frame){
        int h = frame.rows;
        int w = frame.cols;
        cv::Mat mapx,mapy;
        cv::initUndistortRectifyMap(camera_matrix, dist_coeffs,cv::Mat(),camera_matrix,cv::Size(w,h),CV_32FC1,mapx,mapy);
        cv::remap(frame,frame,mapx,mapy,cv::INTER_LINEAR);
    };

private:
    std::string engine_name;
    IRuntime *runtime;
    ICudaEngine *engine;
    IExecutionContext *context;
    char *trtModelStream = nullptr;
    cudaStream_t m_CudaStream;
    int m_InputBindingIndex;
    std::vector<void *> m_DeviceBuffers;
    float out_put[OUTPUT_SIZE];
    void *buffers[2];

    unsigned char *d_resized_img;
    float *d_norm_img;
};

#endif
