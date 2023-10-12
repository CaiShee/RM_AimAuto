#include "WMInference.hpp"
int NUM_CLASSES_ = 1;
int NUM_KEYPOINTS_ = 4;
float IMG_WIDTH_ = 416.;
float IMG_HEIGHT_ = 416.;
cv::Mat letterbox_WM(const cv::Mat &source)
{
    int col = source.cols;
    int row = source.rows;
    int _max = MAX(col, row);
    cv::Mat result = cv::Mat::zeros(_max, _max, CV_8UC3);
    source.copyTo(result(cv::Rect(0, 0, col, row)));
    // result.convertTo(result, CV_32F);
    return result;
}
WMDetector::WMDetector() : compiled_model(core.compile_model(addr.model_address + "wm_416_1_int8.xml",
                                                             "AUTO",
                                                             ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY))),
                           infer_request(compiled_model.create_infer_request())
{
}
WMDetector::~WMDetector()
{
}
bool WMDetector::detect(cv::Mat &img, std::vector<WMObject> &objects)
{
    if (img.empty())
    {
        std::cout << "empty input" << std::endl;
        return false;
    }
    // Preprocess the image
    cv::Mat letterbox_img = letterbox_WM(img);
    float scale = letterbox_img.size[0] / IMG_WIDTH_;
    cv::Mat blob = cv::dnn::blobFromImage(letterbox_img, 1.0 / 255.0, cv::Size(IMG_WIDTH_, IMG_HEIGHT_), cv::Scalar(), true);
    auto input_port = compiled_model.input();
    ov::Tensor input_tensor(input_port.get_element_type(), input_port.get_shape(), blob.ptr(0));
    infer_request.set_input_tensor(input_tensor);
    auto beforeTime = std::chrono::steady_clock::now();

    infer_request.infer();
 
    auto afterTime = std::chrono::steady_clock::now();

    auto output = infer_request.get_output_tensor(0);
    auto output_shape = output.get_shape();
    float *data = output.data<float>();
    cv::Mat output_buffer(output_shape[1], output_shape[2], CV_32F, data);
    transpose(output_buffer, output_buffer); //[8400,56]
    float score_threshold = 0.5;
    float nms_threshold = 0.5;
    std::vector<int> labels;
    std::vector<int> class_ids;
    std::vector<float> class_scores;
    std::vector<cv::Rect> boxes;
    std::vector<std::vector<cv::Point2f>> objects_keypoints;
    objects.clear();
    for (int i = 0; i < output_buffer.rows; i++)
    {
        auto row_ptr = output_buffer.row(i).ptr<float>();
        auto bboxes_ptr = row_ptr;
        auto scores_ptr = row_ptr + 4;
        auto kps_ptr = row_ptr + 4 + NUM_CLASSES_;
        float score = *scores_ptr;
        int label = 0;
        for (int i = 1; i < NUM_CLASSES_; i++)
        {
            scores_ptr++;
            // std::cout << *scores_ptr << std::endl;
            if (*scores_ptr > score)
            {
                score = *scores_ptr;
                label = i;
            }
        }
        if (score > score_threshold)
        {
            class_scores.push_back(score);
            class_ids.push_back(label);
            float cx = *bboxes_ptr++;
            float cy = *bboxes_ptr++;
            float w = *bboxes_ptr++;
            float h = *bboxes_ptr++;
            // Get the box
            int left = int((cx - 0.5 * w) * scale);
            int top = int((cy - 0.5 * h) * scale);
            int width = int(w * scale);
            int height = int(h * scale);
            // Get the keypoints
            std::vector<cv::Point2f> keypoints;
            for (int i = 0; i < NUM_KEYPOINTS_; i++)
            {
                float x = (*(kps_ptr + 2 * i)) * scale;
                float y = (*(kps_ptr + 2 * i + 1)) * scale;
                keypoints.push_back(cv::Point2f(x, y));
            }
            boxes.push_back(cv::Rect(left, top, width, height));
            objects_keypoints.push_back(keypoints);
        }
    }
    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, class_scores, score_threshold, nms_threshold, indices);
    objects.clear();
    for (auto &i : indices)
    {
        WMObject obj;
        obj.cls = class_ids[i];
        // std::cout<<obj.cls<<std::endl;
        obj.color = obj.cls / 3;
        obj.cls %= 3;
        for (int j = 0; j < NUM_KEYPOINTS_; j++)
        {
            obj.apex[j] = objects_keypoints[i][j];
        }
        objects.push_back(obj);
    }
    return true;
}