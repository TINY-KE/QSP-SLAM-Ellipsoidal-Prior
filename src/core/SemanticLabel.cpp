#include "include/core/SemanticLabel.h"
#include "src/config/Config.h"

#include <iostream>
#include <set>
const char* GetLabelText_Optimizer(int id)
{
    static const char *coco_classes[] = {"person","bicycle","car","motorcycle","airplane","bus","train",
    "truck","boat","traffic light","fire hydrant","stop sign","parking meter","bench","bird",
    "cat","dog","horse","sheep","cow","elephant","bear","zebra","giraffe","backpack","umbrella",
    "handbag","tie","suitcase","frisbee","skis","snowboard","sports ball","kite","baseball bat",
    "baseball glove","skateboard","surfboard","tennis racket","bottle","wine glass","cup","fork",
    "knife","spoon","bowl","banana","apple","sandwich","orange","broccoli","carrot","hot dog",
    "pizza","donut","cake","chair","couch","potted plant","bed","dining table","toilet","monitor",
    "laptop","mouse","remote","keyboard","cell phone","microwave","oven","toaster","sink",
    "refrigerator","book","clock","vase","scissors","teddy bear","hair drier","toothbrush"};
    if(id >= 0)
        return coco_classes[id];
    else 
        return "Unknown";
}

std::set<std::string> GenerateValidLabelsForDataset(const string& dataset_type)
{

    if(dataset_type == "TUM"){  // 筛选列表
        return std::set<std::string>
        {

        };
    }
    else if(dataset_type == "Real") // 选择列表
    {
        return std::set<std::string>
        {
            // "potted plant", 
            // "bed", 
            // "vase", 
            // "bottle", 
            // "chair", 
            // "couch"

            // Update for DemoGrad
            "book",
            "laptop",
            "keyboard",
            "mouse",
            "backpack",
            "bottle",
            "monitor",
            "cup"
        };
    }
    else 
    {
        return std::set<std::string>
        {
            "bench",
            "chair",
            "couch",
            "potted plant", // to be decided
            "bed",
            "dining table",
            "refrigerator"
        };
    }

}

bool CheckLabelOnGround(int label)
{
    string dataset_type = ORB_SLAM2::Config::Get<string>("Dataset.Type");

    std::set<std::string> validLabels = GenerateValidLabelsForDataset(dataset_type);

    const char* txtLabel = GetLabelText_Optimizer(label);

    if(dataset_type == "ICL-NUIM" || dataset_type == "Real"){
        if( validLabels.find(std::string(txtLabel)) != validLabels.end() )
            return true;
        else 
            return false;
    }
    else // TUM 
    {
        if( validLabels.find(std::string(txtLabel)) == validLabels.end() )  // 注意该列表为筛选列表，不在列表内则有效
            return true;
        else 
            return false;
    }
}
